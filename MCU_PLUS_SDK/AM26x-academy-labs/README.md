# MCU_PLU_SDK based applications


This folder contains the AM26x-add-on-SDK applications that are based on the MCU_PLUS_SDK.

Before proceeding, please make sure you have the following installed:

| Software/Tool | Link|
|:--|:--:|
| MCU_PLUS_SDK (AM263x)| [Download Link](https://www.ti.com/tool/MCU-PLUS-SDK-AM263X) |
| MCU_PLUS_SDK (AM263Px) | [Download Link](https://www.ti.com/tool/MCU-PLUS-SDK-AM263PX) |
| Syscfg | [Download Link](https://www.ti.com/tool/SYSCONFIG#downloads) |
| TI-ARM-CLANG | [Download Link](https://www.ti.com/tool/download/ARM-CGT-CLANG) |
| Code Composer Studio (CCS) | [Download Link](https://www.ti.com/tool/download/CCSTUDIO/) |
| TI UniFlash Tool | [Download Link](https://www.ti.com/tool/UNIFLASH#downloads) |
| Python3 | [Download Link](https://www.python.org/downloads/) |
| OpenSSL (v3.x) | [Download Link](https://openssl-library.org/source/) |

For detailed steps, please follow: [Download and setup MCU_PLUS_SDK and Tools](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/latest/exports/docs/api_guide_am263x/GETTING_STARTED.html#autotoc_md9)


Once you have the above set up, 
1. Navigate to the application 
2. Read the collateral associated with the application.
3. Download and run the application. 

## Steps to use the AM26x-add-on-SDK

 The application naming convention followed is "{module}\_{device_name}\_{core}\_{OS}\-{tool_chain}" for example, "edma_dma_sw_lab_am263px-cc_r5fss0-0_nortos_ti-arm-clang"
 
The applications inside the AM26x-add-on-SDK are CCS projects that can be directly imported inside your CCS workspace. Before doing that, make sure you:  

1. Install the MCU_PLUS_SDK along with the required Tools. Follow: [Download and setup MCU_PLUS_SDK and Tools](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/latest/exports/docs/api_guide_am263x/GETTING_STARTED.html#autotoc_md9) for a detailed step-wise guide.

2. Download and unzip/Clone the AM26x-add-on-SDK inside the *<mcu_plus_sdk_install_path>/examples* folder.

3. Refer the Readme file of the application you are interested in to know the steps to import and run the application.

---


