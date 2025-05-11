from setuptools import find_packages, setup
import os
import glob

# import logging
# logging.basicConfig(level=logging.INFO) # Set the logging level
# log = logging.getLogger(__name__)

def package_files(data_files, directory_list, dest_path):

    paths_dict = {}

    for directory in directory_list:
        
        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join(dest_path, path)
                
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                    
                else:
                    paths_dict[install_path] = [file_path]
                
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

package_name = 'taro'

supporting_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name +'/launch', ['launch/taro.launch.py', 'launch/taro.sim.launch.py']),
]

supporting_files = package_files(supporting_files, ['runs'], 'lib')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=supporting_files,
    # data_files=package_files(data_files, ['runs/']),
    install_requires=['setuptools', 'ros_numpy', 'ultralytics', 'cv2'],
    zip_safe=True,
    maintainer='azazool',
    maintainer_email='sg.gutierrez95@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'taro_node = taro.taro_node:main'
        ],
    },
)
