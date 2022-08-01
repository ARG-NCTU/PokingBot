import gdown
from zipfile import ZipFile
import os
import shutil

# os.makedirs("./catkin_ws/src/pokingbot_ros/model")
# os.makedirs("./catkin_ws/src/pokingbot_ros/model/goal")

# # download pokingbot model
# dataset_url = 'https://drive.google.com/uc?id=' + "1vfUtkI_RsKfIjmXwdeTdhn7CCcazIhLc"
# dataset_name = "/pokingbot.zip"
# path = os.path.join("./catkin_ws/src/pokingbot_ros/model" + dataset_name)
# gdown.download(dataset_url, output = path, quiet=False)
# zip_file = ZipFile(path)
# zip_file.extractall() # depends on how to zip it
# zip_file.close()
# shutil.move("./policy", "./catkin_ws/src/pokingbot_ros/model")

# # download rl_oa model
# dataset_url = 'https://drive.google.com/uc?id=' + "1fqrIDN86q8xMOqsdq1S96dtwsdZ50gLp"
# dataset_name = "/rl_oa.zip"
# path = os.path.join("./catkin_ws/src/pokingbot_ros/model/goal" + dataset_name)
# gdown.download(dataset_url, output = path, quiet=False)
# zip_file = ZipFile(path)
# zip_file.extractall() # depends on how to zip it
# zip_file.close()
# shutil.move("./policy", "./catkin_ws/src/pokingbot_ros/model/goal")

# download goal yaml
dataset_url = 'https://drive.google.com/uc?id=' + "1OPnIktgxAhfqkBWskPRorzmday7vkvf-"
dataset_name = "goal.yaml"
path = os.path.join("./catkin_ws/src/pokingbot_ros/" + dataset_name)
gdown.download(dataset_url, output = path, quiet=False)