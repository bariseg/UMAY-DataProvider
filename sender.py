import time
import math
import serial
import serial.tools.list_ports
import telemetry_pb2  # Oluşturduğumuz proto dosyası

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
        selected_port = input(f"Kullanılacak Portu Yazın ({', '.join(available_ports)}): ").upper()

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
        altitude = 10.0
        battery = 16.8 # 4S Lipo dolu

        while True:
            # 3. Veriyi Oluştur (Daire çizen İHA simülasyonu)
            flight_data = telemetry_pb2.FlightData()
            
            # Koordinat hesapla (Daire çizme)
            flight_data.latitude = center_lat + (math.sin(angle) * radius)
            flight_data.longitude = center_lon + (math.cos(angle) * radius)
            
            # Diğer verileri simüle et
            flight_data.altitude = altitude + (math.sin(angle*5) * 2) # Hafif dalgalanma
            flight_data.speed = 12.5 # m/s
            
            # Heading (Yön) hesabı: Dairenin teğetine bakar
            heading_deg = math.degrees(angle) % 360
            flight_data.heading = heading_deg 
            
            flight_data.battery = battery
            flight_data.timestamp = int(time.time() * 1000) # Milisaniye cinsinden zaman

            # 4. Veriyi Serileştir (Serialize)
            # Protobuf veriyi binary (byte dizisi) haline getirir
            serialized_data = flight_data.SerializeToString()

            # 5. Gönder
            ser.write(serialized_data)
            
            # Bilgi yazdır
            print(f"Gonderildi ({len(serialized_data)} byte) -> "
                  f"Lat: {flight_data.latitude:.5f}, "
                  f"Lon: {flight_data.longitude:.5f}, "
                  f"Alt: {flight_data.altitude:.1f}m")

            # Simülasyonu ilerlet
            angle += 0.05
            battery -= 0.001 # Pil azalıyor
            if battery < 13.0: battery = 16.8 # Pil bitince fulle

            # 6. Bekleme (Çok Önemli!)
            # Transparent modda veri paketlerinin birbirine yapışmaması için
            # en basit yöntem kısa bir süre beklemektir.
            time.sleep(0.2) # Saniyede 5 veri (5 Hz)

    except serial.SerialException as e:
        print(f"SERİ PORT HATASI: {e}")
    except KeyboardInterrupt:
        print("\nProgram durduruldu.")
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()