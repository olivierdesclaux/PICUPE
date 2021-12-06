# PICUPE

## Pediatric Intensive Care Unit Pose Estimation


This repo will hold the entire pipeline for creating the PICUPE dataset.


## Installation prerequisities
- WE WORK WITH PYTHON=3.8
- To start working on this project, we recommend installing all the dependencies stored in the `environment.yml` file via conda. Open an Anaconda prompt, navigate to the current directory and type 
```
conda env create -f environment.yml
```

### Opensim
1. Download Opensim 4.3. at [this link](https://simtk.org/frs/download_confirm.php/file/6406/OpenSim-4.3-win64.exe?group_id=91).

1. Add `C:\Opensim 4.3\bin` to your environment path ([example here](https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/)).
1. Open an anaconda prompt and activate picupe38
1. Go to C:\Opensim 4.3\sdk\Python and enter the next two following commands:
```
(picupe38) C:\OpenSim 4.3\sdk\Python> python setup_win_python38.py
(picupe38) C:\OpenSim 4.3\sdk\Python> python -m pip install .
```
- Check that opensim is installed by opening a Python shell (type `python`in the command line) and then `import opensim`. If opensim was correctly installed, nothing should happen.
- In case of problems try uninstalling and reinstalling numpy using pip. Also, you can check the instructions given [here](https://simtk-confluence.stanford.edu/display/OpenSim/Scripting+in+Python) and go under 'Setting up your Python environment'


CAUTION: if you decide to install another version of Opensim (e.g. Opensim 4.2.), procedures might change. Check the instructions given here[here](https://simtk-confluence.stanford.edu/display/OpenSim/Scripting+in+Python) and go under 'Setting up your Python environment'

### Microsoft Azure SDK
1. Install [Azure Kinect SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md).
1. Add  `C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\bin` to the Path.
1. If Visual Studio is not yet installed, download [Microsoft Visual C++ 14.0](https://visualstudio.microsoft.com/visual-cpp-build-tools) by selecting the "C++ build tools" option.
1. Open an anaconda prompt and activate picupe38
1. Install `pyk4a` separately using the following command:
    ```
    pip install pyk4a --no-use-pep517 --global-option=build_ext --global-option="-IC:\Program Files\Azure Kinect SDK v1.4.1\sdk\include" --global-option="-LC:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\lib"
    ```

![image de l'installation](https://docs.microsoft.com/en-us/answers/storage/attachments/34873-10262.png "")


### XSens Device API
1. Install the Xsens Device API [here](https://www.xsens.com/cs/c/?cta_guid=ead8a68c-79b8-4b40-bb6d-2f39a1f9847a&signature=AAH58kFF9fy3d8clyHXlTOlQyxqp_XtHxg&pageId=27796161161&placement_guid=df26b080-cbde-4fb7-a9b1-7a9351551530&click=46696b6a-e48e-4dab-9984-c811858a066c&hsutk=c913e22f58d0a8fee21bc3e6682ab9c0&canon=https%3A%2F%2Fwww.xsens.com%2Fsoftware-downloads&utm_referrer=https%3A%2F%2Fwww.google.com%2F&portal_id=3446270&redirect_url=APefjpEW9Fh9DzlHOT0EBIuDptsuZ5hHy9CBVck8Cem5_DLozOkcbdHQIr0-jbYPSlt-qZx5xgjeekbDhLr0Khiz5VzzOeSHTk1pYLv2frKJPWGu8lTHk_2LYADD0la9MHuCRB3K9pU_V2JfU8f8ZYBd40oI1VY4EfJhuTUpuWPVPCq_DqIJVWqPJuV-eEyXjIrc9THQepFhprusS_gbHxBQT60n2inOET8XJG4pN89TUzL02AGlNTYoFNiE5AxrEgi7_TLkiEKZXP5_wIjzXUpiJoyM3X6cAiLidxh_0X2pKkjH3-ykrnpVljyK_Wg8C7ipk5pytlx9eSzwSUQCLmTZKZsFMsWathHjqkXLW0NpqLQhhYxvzlb-a6evN-UXgd25t0_FWqF33zQymLU2kZwUMTs49t80gA&__hstc=81749512.c913e22f58d0a8fee21bc3e6682ab9c0.1620148261255.1627567868128.1627751524463.19&__hssc=81749512.2.1627751524463&__hsfp=2979643178&contentType=standard-page)
1. This will give you the 2019 version. However, we work with the 2021. Go `C:\Program Files\Xsens\MT Software Suite 2019.1.1\MT Manager` and launch the mtmanager64 application
1. Once the Manager is open, in the taskbar: Help > Updates > Check for new MT Software Suite Versions
1. Select version 2021.0.0 and click download and install
When install is finished, in your XSens folder, you should now have two folders: 
	- MT Software Suite 2019.1.1
	- MT Software Suite 2021.0 --> This is what we will be working with
1. Open an anaconda prompt and activate picupe38 and navigate to: `C:\Program Files\Xsens\MT Software Suite 2021.0\MT SDK\Python\x64`. Then type in the command:
```
pip install xsensdeviceapi-2021.0.0-cp38-none-win_amd64.whl
```
Check `C:\Program Files\Xsens\MT Software Suite 2021.0\MT SDK\Examples\xda_python\README.md` for more detailed explanations. 


