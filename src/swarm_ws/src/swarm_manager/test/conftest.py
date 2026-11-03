import os


try:
    _domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
except ValueError:
    _domain = 0
if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not 190 <= _domain <= 220:
    os.environ['ROS_DOMAIN_ID'] = str(190 + os.getpid() % 31)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
