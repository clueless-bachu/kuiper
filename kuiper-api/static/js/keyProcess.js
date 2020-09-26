window.addEventListener("keydown", checkKeyPress, false);
window.addEventListener("keyup", resetKey, false);

urll = "http://35.236.229.125/mission_key"
function changeText(letter)
{
	document.getElementById("key").innerHTML = (letter);
};

function checkKeyPress(key)
{
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

	$.ajax({
		url: urll,
            type: "POST",
            contentType: "application/json",
            data: JSON.stringify({"letter": letter})
        		}).done(function(data) {
            console.log(data); 
           });

};

function resetKey(key) {
	changeText(" ")

	$.ajax({
		url: urll,
            type: "POST",
            contentType: "application/json",
            data: JSON.stringify({"letter": "0"})
        		}).done(function(data) {
            console.log(data); 
           });
};
