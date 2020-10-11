# Import all libraries
import sys
# Comment this line out and replace if required for root privelages
sys.path.append('/home/vasista1997/.local/lib/python3.7/site-packages')
from flask import Flask, request, render_template, redirect, url_for, jsonify

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

	with open('letter', 'w') as out_file: 
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

if __name__ == '__main__':
	app.run(host='0.0.0.0', debug = False, port = 80)
