// Event Listeners that respond to changes in Keyboard command
window.addEventListener("keydown", checkKeyPress, false);
window.addEventListener("keyup", resetKey, false);

// Appropriaterly set the URL
 urll = "http://35.236.229.125/mission_key"
// urll = "http://127.0.0.1:5000/mission_key"

// To limit the number of API POST commands, the code keeps track of time and allows posts at 1Hz
// This prevents lag etween the user and robot which helps in real-time control
var d = new Date();
var process_time = d.getTime();

function changeText(letter)
{ 
	// Changes the text shown on webpage based on the letter key pressed
	document.getElementById("key").innerHTML = (letter);
};

function checkKeyPress(key)
{
	// Based on the key pressed, it changes the display on the webpage and 
	// sends the POST command to the main Flask server
	var d = new Date();
	var n = d.getTime();
	var letter = key.keyCode;
	if(letter =="65")
	{
		changeText("moving left");
	}
	else if(letter =="87")
	{
		changeText("moving forward");
	}
	else if(letter =="68")
	{
		changeText("moving right");
	}
	else if(letter =="83")
	{
		changeText("braking");
	}
	else if(letter =="88")
	{
		changeText("moving backwards");
	}
	else if(letter =="90")
	{
		changeText("Moving back-left");
	}
	else if(letter == "67")
	{
		changeText("Moving back-right");
	}

	
	if(n - process_time>1000) {
		$.ajax({
		url: urll,
            type: "POST",
            contentType: "application/json",
            data: JSON.stringify({"letter": letter})
        		}).done(function(data) {
           });
        process_time = n;
	}

};

function resetKey(key) {

	// If key is released, it changes the display on the webpage and 
	// sends the POST command to the main Flask server
	changeText("Not Moving")
	$.ajax({
		url: urll,
            type: "POST",
            contentType: "application/json",
            data: JSON.stringify({"letter": "0"})
        		}).done(function(data) {
           });

    
};
