window.addEventListener("keydown", checkKeyPress, false);
window.addEventListener("keyup", resetKey, false);

// urll = "http://35.236.229.125/mission_key"
 urll = "http://127.0.0.1:5000/mission_key"
var d = new Date();
var process_time = d.getTime();
function changeText(letter)
{
	document.getElementById("key").innerHTML = (letter);
};

function checkKeyPress(key)
{
	var d = new Date();
	var n = d.getTime();
	var letter = key.keyCode;
	if(letter =="65")
	{
		changeText("a");
	}
	else if(letter =="87")
	{
		changeText("w");
	}
	if(letter =="68")
	{
		changeText("d");
	}
	else if(letter =="83")
	{
		changeText("s");
	}

	console.log(n - process_time);
	if(n - process_time>1000) {
		$.ajax({
		url: urll,
            type: "POST",
            contentType: "application/json",
            data: JSON.stringify({"letter": letter})
        		}).done(function(data) {
            // console.log(data); 
           });
        process_time = n;
	}
	

     // setTimeout(pollDOM, 300);

};

function resetKey(key) {
	changeText(" ")
	$.ajax({
		url: urll,
            type: "POST",
            contentType: "application/json",
            data: JSON.stringify({"letter": "0"})
        		}).done(function(data) {
            // console.log(data); 
           });

    
};


function pollDOM() {
;
};