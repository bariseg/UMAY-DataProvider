import time
import serial
import struct
import random
import telemetry_pb2  # Protobuf derlemesinden çıkan dosya

# --- AYARLAR ---
# Linux/Raspberry Pi için genelde: '/dev/ttyUSB0' veya '/dev/ttyS0'
# Windows'ta test ediyorsan: 'COM4'
XBEE_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 9600

def main():
    try:
        # XBee Seri Port Bağlantısını Başlat
        ser = serial.Serial(
            port=XBEE_PORT,
            baudrate=BAUD_RATE,
            timeout=1
        )
        print(f"XBee Baglantisi Baslatildi: {XBEE_PORT} @ {BAUD_RATE}")
        
        while True:
            # 1. Veri Oluştur (Simülasyon)
            # Gerçek senaryoda buraya sensör okuma fonksiyonlarını koyacaksın.
            telemetry = telemetry_pb2.TelemetryData()
            
            # Örnek İHA verileri (İstanbul üzerinde uçuyor gibi)
            telemetry.latitude = 41.015137 + (random.uniform(-0.001, 0.001))
            telemetry.longitude = 28.979530 + (random.uniform(-0.001, 0.001))
            telemetry.altitude = 50.0 + random.uniform(-1, 1)
            telemetry.speed = 15.0 + random.uniform(-0.5, 0.5)
            telemetry.battery = 14.8 - (0.001) # Pil azalıyor simülasyonu
            telemetry.heading = random.uniform(0, 360)
            
            # 2. Veriyi Serileştir (Byte dizisine çevir)
            data_bytes = telemetry.SerializeToString()
            
            # 3. Paketleme (Framing) - ÖNEMLİ!
            # Alıcı tarafın paketin nerede başlayıp bittiğini anlaması için
            # verinin başına verinin uzunluğunu (2 byte) ekliyoruz.
            # '>H' = Big-Endian Unsigned Short (2 byte tamsayı)
            packet_size = len(data_bytes)
            header = struct.pack('>H', packet_size)
            
            final_packet = header + data_bytes
            
            # 4. Veriyi Gönder
            ser.write(final_packet)
            
            print(f"Gonderildi: {packet_size} byte veri + 2 byte baslik.")
            
            # Gönderim sıklığı (Hz)
            time.sleep(0.1) # 10 Hz (Saniyede 10 veri)

    except serial.SerialException as e:
        print(f"HATA: Seri port hatasi -> {e}")
    except KeyboardInterrupt:
        print("Program durduruluyor...")
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()