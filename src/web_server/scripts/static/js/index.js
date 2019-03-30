
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

var update_info_btn = document.getElementById("update_info_btn");

var experiment_ip_input = document.getElementById("experiment_ip");
experiment_ip_input.value = "192.168.100.102";

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
    getJSON("http://" + experiment_ip_input.value + "/activate", ()=>{});
}

experiment_deactivate_action_btn.onclick = function()
{
    getJSON("http://" + experiment_ip_input.value + "/deactivate", ()=>{});
}

update_info_btn.onclick = function()
{
    getJSON(window.location + "/update_info", ()=>{});
}