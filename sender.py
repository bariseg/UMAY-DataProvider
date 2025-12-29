import time
import math
import serial
import struct
import serial.tools.list_ports
import telemetry_pb2
from pymavlink import mavutil # MAVLink kütüphanesi eklendi

# --- AYARLAR ---
XBEE_BAUD_RATE = 9600      # Telemetri modülü hızı
PIXHAWK_BAUD_RATE = 57600  # Pixhawk genelde USB'de otomatik tanır ama Telem portu ise 57600/115200 olabilir.

# Pixhawk bağlantı stringi (Raspberry Pi'de '/dev/ttyACM0' veya '/dev/ttyAMA0' olabilir)
# Windows için 'COMx', Mac için '/dev/cu.usbmodem...'
PIXHAWK_PORT_STR = 'COM9' # <-- BURAYI PIXHAWK PORTU İLE GÜNCELLE

def list_serial_ports():
    """Bilgisayardaki aktif portları listeler."""
    ports = serial.tools.list_ports.comports()
    print("\n--- Bulunan Seri Portlar ---")
    for port, desc, hwid in ports:
        print(f"{port}: {desc}")
    print("----------------------------\n")
    return [p.device for p in ports]

def connect_pixhawk(port):
    """Pixhawk'a MAVLink üzerinden bağlanır ve veri akışını ister."""
    print(f"Pixhawk aranıyor: {port}...")
    try:
        # device=port, baud=baud_rate. autoreconnect=True bağlantı kopsa da dener.
        master = mavutil.mavlink_connection(port, baud=PIXHAWK_BAUD_RATE, autoreconnect=True)
        master.wait_heartbeat() # İlk kalp atışını bekle
        print(f"Pixhawk Bağlandı! Sistem ID: {master.target_system}, Bileşen ID: {master.target_component}")

        # Pixhawk'tan veri akışını başlatmasını iste (request_data_stream)
        # Tüm verileri (MAV_DATA_STREAM_ALL) 4 Hz (saniyede 4 kere) iste
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4, # Hız (Hz)
            1  # Başlat (1) / Durdur (0)
        )
        return master
    except Exception as e:
        print(f"Pixhawk bağlantı hatası: {e}")
        return None

def main():
    # 1. Port Seçimi (XBEE İÇİN)
    print("--- XBEE PORT SEÇİMİ ---")
    available_ports = list_serial_ports()
    
    if not available_ports:
        print("HATA: Hiçbir COM portu bulunamadı!")
        return

    # Otomatik seçim mantığı veya manuel giriş
    # Not: Gerçek senaryoda burayı hardcoded yapabilirsin örn: '/dev/ttyUSB0'
    if len(available_ports) == 1:
        xbee_port = available_ports[0]
    else:
        # Şimdilik listedeki ilk boşta olanı veya manuel seçimi kullanabilirsin
        # Pixhawk portu ile çakışmamasına dikkat et!
        xbee_port = 'COM5' # input(f"XBee Portunu Yazın: ").upper()

    print(f"XBee Hedef: {xbee_port} @ {XBEE_BAUD_RATE}")

    # Verileri saklamak için son durum değişkenleri (Data Caching)
    # MAVLink mesajları asenkron gelir, hepsini tek seferde alamayabiliriz.
    current_data = {
        'lat': 0.0, 'lon': 0.0, 'alt': 0.0,
        'speed': 0.0, 'heading': 0.0,
        'battery': 0.0, 'roll': 0.0, 'pitch': 0.0
    }

    try:
        # 2. Bağlantıları Başlat
        # A) XBee (Serial)
        ser_xbee = serial.Serial(xbee_port, XBEE_BAUD_RATE, timeout=1)
        
        # B) Pixhawk (MAVLink)
        mav_master = connect_pixhawk(PIXHAWK_PORT_STR)
        if not mav_master:
            print("Pixhawk bulunamadı, çıkılıyor...")
            return

        print("Sistem Hazır. Veri akışı başlıyor...\n")

        last_send_time = time.time()
        
        while True:
            # --- ADIM 1: Pixhawk'tan Veri Oku ---
            # Blocking=False ile döngüyü kilitlemeden mesaj var mı diye bakarız
            msg = mav_master.recv_match(blocking=False)
            
            if msg:
                msg_type = msg.get_type()
                
                # Konum Verisi (GLOBAL_POSITION_INT)
                if msg_type == 'GLOBAL_POSITION_INT':
                    # MAVLink lat/lon verisini integer (1E7) gönderir, float'a çeviriyoruz.
                    current_data['lat'] = msg.lat / 1e7
                    current_data['lon'] = msg.lon / 1e7
                    # relative_alt (kalkış noktasına göre) mm cinsindendir -> metreye çevir
                    current_data['alt'] = msg.alt / 1000.0
                    # hdg (heading) cdeg (centidegree) -> dereceye çevir
                    current_data['heading'] = msg.hdg / 100.0

                # Hız ve Hava Verisi (VFR_HUD)
                elif msg_type == 'VFR_HUD':
                    current_data['speed'] = msg.groundspeed # veya msg.airspeed

                # Açısal Durum (ATTITUDE)
                elif msg_type == 'ATTITUDE':
                    # Radyan gelir, dereceye çevirmeliyiz
                    current_data['roll'] = math.degrees(msg.roll)
                    current_data['pitch'] = math.degrees(msg.pitch)

                # Batarya Durumu (SYS_STATUS)
                elif msg_type == 'SYS_STATUS':
                    # voltage_battery milivolt cinsindendir -> volt yap
                    current_data['battery'] = msg.voltage_battery / 1000.0

            # --- ADIM 2: Belirli Aralıklarla Telemetri Gönder (Örn: 5Hz - 0.2sn) ---
            if time.time() - last_send_time > 0.2:
                
                # Proto nesnesini doldur
                flight_data = telemetry_pb2.FlightData()
                flight_data.latitude = current_data['lat']
                flight_data.longitude = current_data['lon']
                flight_data.altitude = current_data['alt']
                flight_data.speed = current_data['speed']
                flight_data.heading = current_data['heading']
                flight_data.battery = current_data['battery']
                flight_data.timestamp = int(time.time() * 1000)
                flight_data.roll = current_data['roll']
                flight_data.pitch = current_data['pitch']

                # Serileştirme ve Paketleme (Framing)
                serialized_data = flight_data.SerializeToString()
                length_prefix = struct.pack('>I', len(serialized_data))
                final_packet = length_prefix + serialized_data

                # XBee üzerinden gönder
                ser_xbee.write(final_packet)
                
                print(
                    f"[TX] Lat:{flight_data.latitude:.6f} Lon:{flight_data.longitude:.6f} "
                    f"Alt:{flight_data.altitude:.1f}m Bat:{flight_data.battery:.1f}V "
                    f"R:{flight_data.roll:.0f} P:{flight_data.pitch:.0f}"
                )
                
                last_send_time = time.time()

    except serial.SerialException as e:
        print(f"SERİ PORT HATASI: {e}")
    except KeyboardInterrupt:
        print("\nProgram durduruldu.")
        if 'ser_xbee' in locals() and ser_xbee.is_open:
            ser_xbee.close()
        if 'mav_master' in locals():
            mav_master.close()

if __name__ == "__main__":
    main()