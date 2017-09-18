# 1 Introduction

BLE Hello Sensor/Center demos show how to establish a BLE Connection and communicate.

# 2 How to use BLE Sensor or Center demos

Two AZ3239-Kits at least to demonstrate demos.

## (1) Run ble_hello_center demo in an AZ3239-Kit

### Change main.cpp as follow

```c
int main( )
{
    /* APPLICATION can be assigned to the folder names under folder "APP" */
//    RUN_APPLICATION( iperf );
//    RUN_APPLICATION( sirc_transmit );
//    RUN_APPLICATION( mbed_wifi );
//    RUN_APPLICATION( mbed_tls_client );
//    RUN_APPLICATION( mbed_tcp_udp );
//    RUN_APPLICATION( soft_ap );
//    RUN_APPLICATION( audio );
//    RUN_APPLICATION( ble_hello_sensor );
    RUN_APPLICATION( ble_hello_center );
}
```
### Build and download and run

```
  $$ mbed compile -t GCC_ARM -m AZ3239
```

Copy `mbed-wifi-example.git.bin` to an AZ3239-Kit Driver on Windows.

## (2) Run ble_hello_sensor demo in another AZ3239-Kit

### Change main.cpp as follow

```c
int main( )
{
    /* APPLICATION can be assigned to the folder names under folder "APP" */
//    RUN_APPLICATION( iperf );
//    RUN_APPLICATION( sirc_transmit );
//    RUN_APPLICATION( mbed_wifi );
//    RUN_APPLICATION( mbed_tls_client );
//    RUN_APPLICATION( mbed_tcp_udp );
//    RUN_APPLICATION( soft_ap );
//    RUN_APPLICATION( audio );
    RUN_APPLICATION( ble_hello_sensor );
//    RUN_APPLICATION( ble_hello_center );
}
```

### Build and download and run 

See details above.

## (3) Establish connection over BLE and communicate

If both demos run successfully, the Center device will automatically scan and connect to the Sensor. When the connection is successful, the Center sends a Color value to every connected Sensor, and the RGB LED color value of the Sensor should changes.



