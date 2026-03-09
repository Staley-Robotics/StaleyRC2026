import os
import sys
import platform

def run_command(command:str, initializing_output:str, failure_output:str) -> None:
   print(initializing_output)

   exit_code = os.system(command)
   if exit_code == 0:
      print("Success!")
   else:
      raise Exception(f"{failure_output}. Recieved Error '{exit_code}'")

#Detect platform
platform_name = platform.system()


print("\nWelcome to the terrible setup wizard!")
if not input("This will: create a python virtual environment, install pip requirements, and run robotpy sync. It will require internet access. Proceed? [y/n]: ").lower().strip() in ("y", "ye", "yes"):
   print("Exiting.")
   exit()


'''-------------------Virtual Environment-------------------'''
print("\nChecking for Python Virtual Environment...")

if hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix): #<-
   print("Virtual environment detected as active")
elif ".venv" in os.listdir():
   raise Exception("There appears to be a venv in this location, but it is not active, please try restarting your terminal/VSCode.")
else:
   print("No virtual environment is active, and none was detected in the workspace")
   run_command("python3 -m venv .venv", "Creating python venv...", "Encountered error while creating virtual environment")
   run_command("source .venv/bin/activate", "Activating virtual environment...", "Encountered error while activating virtual environment")
   # print("Please close your terminal and rerun this file to load the venv") #I should be able to load it by calling it through os, but there's problems
   # exit(0)


'''-------------------pip install-------------------'''
print("\nInstalling project requirements...")
if "requirements.txt" in os.listdir():
   print("requirements.txt detected")
   run_command("pip install -r requirements.txt", "installing requirements.txt...", "Encountered error while installing requirements.txt")
   if platform_name == "Darwin":
      run_command("pip install certifi", "Installing certifi for MacOS...", "Encountered error while installing certifi")
else:
   raise Exception("Project requirements file (requirements.txt) not detected. Please find it!")


'''-------------------robotpy-------------------'''
if platform_name == "Darwin": #Check if on MacOS, if so, assume is a school mac, so need certifi to certify
   run_command("robotpy sync --use-certifi", "\nSyncing RobotPy (MacOS detected)...", "Encountered error while running 'robotpy sync --use-certifi'")
else:
   run_command("robotpy sync", "\nSyncing RobotPy...", "Encountered error while running 'robotpy sync'")

'''-------------------complete-------------------'''
print("\nEnvironment Succesfully Setup!")