# Import all libraries
import sys
# Comment this line out and replace if required for root privelages
sys.path.append('/home/vasista1997/.local/lib/python3.7/site-packages')
from flask import Flask, request, render_template, redirect, url_for, jsonify, Response
import cv2
import numpy as np
import os
from time import sleep


# create flask app
app = Flask(__name__)

@app.route('/')
def home():
    '''
    Defines the home page
    '''
    return render_template('home.html')

@app.route('/login/', methods= ['POST', 'GET'])
def login():
    '''
    Defines the login page. Simple login where you can post you credentials and required login action 
    is taken

    REQUIRED: Change credentials
    '''
    if request.method == 'GET':
        return render_template('login.html')
    elif request.method == 'POST':
        name = request.form['nm']
        if name == 'vasista':
            return redirect(url_for('mission_control'))
        else:
            return render_template('login.html', base_message = 'Login Failed, Try Again')


@app.route('/mission_control')
def mission_control():
    '''
    The main webpage where the Kuiper bot is controlled
    '''
    return render_template('mission_control.html')

@app.route('/mission_key', methods = ['POST'])
def mission_key():
    '''
    Accesses the key from an external POST source (Here the ajax command in the js script and writes it
    to a simple database)
    '''
    data = request.get_json()

    with open('./database/letter', 'w') as out_file: 
        out_file.write(str(data['letter']))
    return jsonify({'response': data})


@app.route('/key_parser')
def key_parser():
    '''
    Handles API requests by reading the current robot commands from the database and returning a json file
    '''
    with open('letter', 'r') as f: 
        letter = f.read()
    return jsonify({'response': letter})


def gen_frames():  # generate frame by frame from camera
    '''
    generates a single image per call to be used for streaming. If streaming not available, then uses default
    'no video available' tag
    '''
    img = cv2.imread("./database/no_vid.PNG")
    img = cv2.resize(img, (100,100))
    _, default_buffer = cv2.imencode('.jpg', img)
    while True:
        sleep(0.21)
        try:        
            buffer = np.load('./database/comp_img.npy')
        except: # if image is not available display the 'video not available tag'
            buffer = default_buffer

        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
     

@app.route('/compressed_image', methods = ['POST'])
def compressed_image():
    '''
    Accesses the compressed image from an external POST source from the robot
    '''
    r = request
    nparr = np.frombuffer(r.data, np.uint8).T
    np.save('./database/comp_img', nparr)
    return jsonify({'response': "recieved image"})

@app.route('/video_feed')
def video_feed():
    '''
    Video streaming route. This is used as the src attribute of an img tag which gets a continuous stream
    '''
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug = True)

    if os.path.exists("./database/comp_img.npy"):
        os.remove("./database/comp_img.npy")
    # host='0.0.0.0', debug = False, port = 80
