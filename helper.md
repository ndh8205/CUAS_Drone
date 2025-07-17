```bash
sudo apt update
sudo apt install python3-pip
pip3 install pyserial pynmea2
```

elrs_gps.py

```bash

import serial
import time
import pynmea2
import struct

# --- 설정 ---
# 배선에 따라 포트 이름이 다를 수 있음 (ls /dev/ttyAMA* 명령으로 확인)
GPS_PORT = "/dev/ttyAMA2"  # GPS 모듈이 연결된 UART 포트
ELRS_PORT = "/dev/ttyAMA1" # ELRS 수신기가 연결된 UART 포트

GPS_BAUDRATE = 9600
ELRS_BAUDRATE = 420000

# CRSF CRC8-DVB-S2 다항식
def crsf_crc8(data):
    poly = 0xD5
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
    return crc & 0xFF

def send_gps_frame(ser, lat, lon, gspd, head, alt, sats):
    # 페이로드 생성
    # 형식: lat(int32), lon(int32), gspd(uint16), head(uint16), alt(uint16), sats(uint8)
    payload = struct.pack('<iihhHHB', 
                          int(lat * 1e7), 
                          int(lon * 1e7), 
                          int(gspd * 3.6 * 10), # m/s를 cm/s로 변환 후, 스펙에 따라 km/h*10으로 가정
                          int(head * 100), 
                          int(alt + 1000), # 1000m 오프셋
                          sats)

    # 프레임 생성: [주소, 길이, 타입] + 페이로드
    frame_type = 0x02 # GPS 프레임 타입
    frame_header = [0xC8, len(payload) + 2, frame_type] # 주소: C8(FC)
    full_frame = bytearray(frame_header) + payload

    # CRC 계산 및 추가
    crc = crsf_crc8(full_frame[2:]) # 타입부터 페이로드 끝까지 CRC 계산
    full_frame.append(crc)

    # ELRS로 전송
    ser.write(full_frame)
    print(f"Sent GPS: Lat={lat}, Lon={lon}, Sats={sats}")

def main():
    try:
        gps_ser = serial.Serial(GPS_PORT, baudrate=GPS_BAUDRATE, timeout=1.0)
        elrs_ser = serial.Serial(ELRS_PORT, baudrate=ELRS_BAUDRATE)
        print("Serial ports opened successfully.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    while True:
        try:
            line = gps_ser.readline().decode('utf-8', errors='ignore')
            if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                msg = pynmea2.parse(line)
                if isinstance(msg, pynmea2.types.talker.RMC) and msg.status == 'A':
                    send_gps_frame(elrs_ser,
                                   msg.latitude,
                                   msg.longitude,
                                   msg.spd_over_grnd_kts * 0.514444, # knots to m/s
                                   msg.true_course if msg.true_course is not None else 0,
                                   msg.altitude if hasattr(msg, 'altitude') else 0, # RMC에는 고도가 없으므로 GGA 필요
                                   msg.num_sats if hasattr(msg, 'num_sats') else 0  # RMC에는 위성 수가 없음
                                   )

        except (pynmea2.ParseError, UnicodeDecodeError):
            continue
        except Exception as e:
            print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()
	
```	
