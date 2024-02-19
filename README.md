# Matter Barometer Example (ESP32 + BMP180)
Matter 기압계 센서 예제 프로젝트<br>

Software (Matter)
---

Hardware
---
[BMP180]

SDK Version
---


Helper Scripts
---
SDK 클론 및 설치
```shell
$ source ./scripts/install_sdk.sh
```
SDK (idf.py) 준비
```shell
$ source ./scripts/prepare_sdk.sh
```

Build & Flash Firmware
---
1. Factory Partition (Matter DAC)
    ```shell
    $ source ./scripts/flash_factory_dac_provider.sh
    ```
2. Configure project
    ```shell
    $ idf.py set-target esp32
    ```
3. Build Firmware
    ```shell
    $ idf.py build
    ```
4. Flash Firmware
    ```shell
    $ idf.py -p ${seiral_port} flash monitor
    ```

QR Code for commisioning
---
![qrcode.png](./resource/DACProvider/qrcode.png)

References
---
[Matter 압력 측정 클러스터 개발 예제 (ESP32)](https://yogyui.tistory.com/entry/PROJ-Matter-%EC%95%95%EB%A0%A5-%EC%B8%A1%EC%A0%95-%ED%81%B4%EB%9F%AC%EC%8A%A4%ED%84%B0-%EA%B0%9C%EB%B0%9C-%EC%98%88%EC%A0%9C-ESP32)<br>

