import sys
sys.path.append('/home/vasista1997/.local/lib/python3.7/site-packages')
from flask import Flask, request, render_template, redirect, url_for, jsonify

app = Flask(__name__)


@app.route('/')
def home():
	return render_template('home.html')

@app.route('/login/', methods= ['POST', 'GET'])
def login():
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
	return render_template('mission_control.html')

@app.route('/mission_key', methods = ['POST'])
def mission_key():
	data = request.get_json()
	#print(data)

	with open('letter', 'w') as out_file: 
		out_file.write(str(data['letter']))
	return jsonify({'response': data})


@app.route('/key_parser')
def key_parser():
	with open('letter', 'r') as f: 
		letter = f.read()
	return jsonify({'response': letter})

if __name__ == '__main__':
	app.run(host='0.0.0.0', debug = False, port = 80)
