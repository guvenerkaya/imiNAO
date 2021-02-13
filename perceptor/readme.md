*for starting perception part:
-go via 'roscd' command to 'percpetor/backend'
-type 'http-server' which enables server access
-in a webbrowser (preferablly chrome) type : 'localhost:8080/camera.html' (a window for getting the camera frame for NAO opens)
-type 'roslaunch perceptor perceptor.launch' to run all the perception nodes

*for the contoller nodes, see 'readme' in 'control_v2'

*in case of crashing with 'roslaunch nao_bringup (...)'
->how to run the script on the lab workstation?
1 open a terminal
2 activate the anaconda environment
 `conda activate torch`

