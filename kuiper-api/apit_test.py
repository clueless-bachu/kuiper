import requests
import json
from time import sleep

url = "http://127.0.0.1:5000/key_parser"


while(1):
	html_text = json.loads(requests.get(url).text)
	print(html_text['response'])
	sleep(1)