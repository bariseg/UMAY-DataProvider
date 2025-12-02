import time
import math
import serial
import struct # EKLENDI: Paketleme için şart
import serial.tools.list_ports
import telemetry_pb2  # Oluşturduğumuz proto dosyası

#python -m grpc_tools.protoc -I . --python_out . --pyi_out . ./telemetry.proto  

# --- AYARLAR ---
BAUD_RATE = 9600  # XBee modüllerindeki ayarın aynısı olmalı!

def list_serial_ports():
    """Bilgisayardaki aktif portları listeler."""
    ports = serial.tools.list_ports.comports()
    print("\n--- Bulunan Seri Portlar ---")
    for port, desc, hwid in ports:
        print(f"{port}: {desc}")
    print("----------------------------\n")
    return [p.device for p in ports]

def main():
    # 1. Port Seçimi
    available_ports = list_serial_ports()
    if not available_ports:
        print("HATA: Hiçbir COM portu bulunamadı! XBee takılı mı?")
        return

    # Eğer sadece 1 port varsa otomatik seç, yoksa kullanıcıya sor
    if len(available_ports) == 1:
        selected_port = available_ports[0]
    else:
        selected_port = 'COM5' #input(f"Kullanılacak Portu Yazın ({', '.join(available_ports)}): ").upper()

    print(f"Bağlanılıyor: {selected_port} @ {BAUD_RATE}")

    try:
        # 2. Bağlantıyı Başlat
        ser = serial.Serial(selected_port, BAUD_RATE, timeout=1)
        time.sleep(2) # Bağlantının oturması için kısa bekleme
        print("XBee Bağlantısı Başarılı! Veri gönderimi başlıyor...\n")

        # --- SİMÜLASYON DEĞİŞKENLERİ ---
        # Başlangıç: İstanbul / Tarihi Yarımada
        center_lat = 41.0082
        center_lon = 28.9784
        radius = 0.002 # Dairenin çapı
        angle = 0.0
        altitude = 100.0
        battery = 16.8 # 4S Lipo dolu

        while True:
            # 1. Veriyi Hazırla (Tüm .proto alanları dolduruluyor)
            flight_data = telemetry_pb2.FlightData()
            # Konum: merkez etrafında dairesel hareket
            flight_data.latitude = center_lat + (math.sin(angle) * radius)
            flight_data.longitude = center_lon + (math.cos(angle) * radius)
            # Yükseklik: baz yükseklik + açının fonksiyonuna göre dalgalanma
            flight_data.altitude = altitude + (math.sin(angle * 3.0) * 10.0)
            # Hız: küçük osilasyonlarla simüle edilir
            flight_data.speed = 12.5 + (math.sin(angle * 2.0) * 1.0)
            # Yön (0-360)
            flight_data.heading = (math.degrees(angle) % 360)

            # Pil: yavaşça azalır, belli bir eşik altına düşünce yeniden doldurulur (simülasyon amaçlı)
            battery -= 0.01
            if battery < 12.0:
                battery = 16.8
            flight_data.battery = battery

            # Zaman damgası (ms)
            flight_data.timestamp = int(time.time() * 1000)

            # Roll ve pitch: açının fonksiyonuna göre örnek değerler (derece cinsinden)
            flight_data.roll = math.sin(angle * 4.0) * 30.0   # +/-30 derece civarı
            flight_data.pitch = math.cos(angle * 4.0) * 15.0  # +/-15 derece civarı

            # 2. Protobuf ile Serileştir
            serialized_data = flight_data.SerializeToString()
            
            # 3. FRAMING (ÇERÇEVELEME) - KRİTİK NOKTA
            # Mesajın uzunluğunu hesapla (Örn: 56 byte)
            # '>I' formatı: Big-Endian Unsigned Integer (4 byte)
            length_prefix = struct.pack('>I', len(serialized_data))
            
            # Uzunluk bilgisi + Veriyi birleştir
            final_packet = length_prefix + serialized_data

            # 4. Gönder
            ser.write(final_packet)
            
            print(
                f"Gonderildi: {len(serialized_data)} byte veri + 4 byte başlık. "
                f"lat={flight_data.latitude:.6f} lon={flight_data.longitude:.6f} alt={flight_data.altitude:.2f} "
                f"spd={flight_data.speed:.2f} hdg={flight_data.heading:.1f} bat={flight_data.battery:.2f} "
                f"roll={flight_data.roll:.1f} pitch={flight_data.pitch:.1f} ts={flight_data.timestamp}"
            )
            
            angle += 0.05
            time.sleep(0.6) # 10 Hz

    except serial.SerialException as e:
        print(f"SERİ PORT HATASI: {e}")
    except KeyboardInterrupt:
        print("\nProgram durduruldu.")
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()