# PICUPE

## Pediatric Intensive Care Unit Pose Estimation


This repo will hold the entire pipeline for creating the PICUPE dataset.


## Installation prerequisities
- WE WORK WITH PYTHON=3.7
- Need to create a config.yml with all basic installations/packages
	- Python=3.7
	- numpy
	- scipy
	- matplotlib
	- pyopengl
	- simbody


### Opensim
- Download Opensim 4.2. at [this link](https://simtk.org/frs/?group_id=91).
- Then follow instructions given [here](https://simtk-confluence.stanford.edu/display/OpenSim/Scripting+in+Python)
and go under 'Setting up your Python environment'
- You also need to install simbody: conda install -c conda-forge simbody 

### Microsoft Azure SDK
- See README.md given in the Python-calibrate folder

### XSens Device API
- Install the Xsens Device API [here](https://www.xsens.com/cs/c/?cta_guid=ead8a68c-79b8-4b40-bb6d-2f39a1f9847a&signature=AAH58kFF9fy3d8clyHXlTOlQyxqp_XtHxg&pageId=27796161161&placement_guid=df26b080-cbde-4fb7-a9b1-7a9351551530&click=46696b6a-e48e-4dab-9984-c811858a066c&hsutk=c913e22f58d0a8fee21bc3e6682ab9c0&canon=https%3A%2F%2Fwww.xsens.com%2Fsoftware-downloads&utm_referrer=https%3A%2F%2Fwww.google.com%2F&portal_id=3446270&redirect_url=APefjpEW9Fh9DzlHOT0EBIuDptsuZ5hHy9CBVck8Cem5_DLozOkcbdHQIr0-jbYPSlt-qZx5xgjeekbDhLr0Khiz5VzzOeSHTk1pYLv2frKJPWGu8lTHk_2LYADD0la9MHuCRB3K9pU_V2JfU8f8ZYBd40oI1VY4EfJhuTUpuWPVPCq_DqIJVWqPJuV-eEyXjIrc9THQepFhprusS_gbHxBQT60n2inOET8XJG4pN89TUzL02AGlNTYoFNiE5AxrEgi7_TLkiEKZXP5_wIjzXUpiJoyM3X6cAiLidxh_0X2pKkjH3-ykrnpVljyK_Wg8C7ipk5pytlx9eSzwSUQCLmTZKZsFMsWathHjqkXLW0NpqLQhhYxvzlb-a6evN-UXgd25t0_FWqF33zQymLU2kZwUMTs49t80gA&__hstc=81749512.c913e22f58d0a8fee21bc3e6682ab9c0.1620148261255.1627567868128.1627751524463.19&__hssc=81749512.2.1627751524463&__hsfp=2979643178&contentType=standard-page)
- This will give you the 2019 version. However, we work with the 2021. 
- Go into the created XSens directory. Go to Xsens\MT Software Suite 2019.1.1\MT Manager and launch the mtmanager64 application
- Once the Manager is open, in the taskbar: Help > Updates > Check for new MT Software Suite Versions
- Select version 2021.0.0 and click download and install
- When install is finished, in your XSens folder, you should now have two folders: 
	- MT Software Suite 2019.1.1
	- MT Software Suite 2021.0 --> This is what we will be working with
	
- Navigate to: Xsens\MT Software Suite 2021.0\MT SDK\Examples\xda_python and follow instructions given in the README.md


