
var getJSON = function(url, callback) {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', url, true);
    xhr.responseType = 'json';
    xhr.onload = function() {
      var status = xhr.status;
      if (status === 200) {
        callback(null, xhr); // N.B ideally should return xhr.response
      } else {
        callback(status, xhr);
      }
    };
    xhr.send();
};

var gripper_open_action_btn  = document.getElementById("gripper_open_action_btn");
var gripper_close_action_btn = document.getElementById("gripper_close_action_btn");

var experiment_activate_action_btn  = document.getElementById("experiment_activate_action_btn");
var experiment_deactivate_action_btn = document.getElementById("experiment_deactivate_action_btn");

gripper_open_action_btn.onclick = function()
{
    getJSON(window.location.href + "open_gripper", ()=>{});
}

gripper_close_action_btn.onclick = function()
{
    getJSON(window.location.href + "close_gripper", ()=>{});
}

experiment_activate_action_btn.onclick = function()
{
    getJSON("http://192.168.100.102/activate", ()=>{});
}

experiment_deactivate_action_btn.onclick = function()
{
    getJSON("http://192.168.100.102/deactivate", ()=>{});
}