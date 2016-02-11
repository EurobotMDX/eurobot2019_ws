
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

var reset_task_btn = document.getElementById("reset_task_btn");
var eurobot_kill_task_btn = document.getElementById("eurobot_kill_task_btn");
var start_task_purple_btn = document.getElementById("start_task_purple_btn");
var start_task_yellow_btn = document.getElementById("start_task_yellow_btn");

var gripper_open_action_btn  = document.getElementById("gripper_open_action_btn");
var gripper_close_action_btn = document.getElementById("gripper_close_action_btn");

var pusher_left_action_btn  = document.getElementById("pusher_left_action_btn");
var pusher_right_action_btn = document.getElementById("pusher_right_action_btn");

var experiment_activate_action_btn  = document.getElementById("experiment_activate_action_btn");
var experiment_deactivate_action_btn = document.getElementById("experiment_deactivate_action_btn");

var update_info_btn = document.getElementById("update_info_btn");

var robot_x   = document.getElementById("robot_x");
var robot_y   = document.getElementById("robot_y");
var robot_yaw = document.getElementById("robot_yaw");

var reset_odometry_btn = document.getElementById("reset_odometry_btn");

var experiment_ip_input = document.getElementById("experiment_ip");
experiment_ip_input.value = "192.168.100.102";

function update_robot_position()
{
    getJSON(window.location.href + "get_robot_position", (status, xhr)=>{
        var data = xhr.response;
        robot_x.value   = (data.x).toFixed(5);
        robot_y.value   = (data.y).toFixed(5);
        robot_yaw.value = ((data.yaw / Math.PI) * 180.0).toFixed(5);
    });
}

setInterval(update_robot_position, 1000);

reset_odometry_btn.onclick = function()
{
    getJSON(window.location.href + "reset_odometry", ()=>{});
}

gripper_open_action_btn.onclick = function()
{
    getJSON(window.location.href + "open_gripper", ()=>{});
}

gripper_close_action_btn.onclick = function()
{
    getJSON(window.location.href + "close_gripper", ()=>{});
}

pusher_left_action_btn.onclick = function()
{
    getJSON(window.location.href + "push_left", ()=>{});
}

pusher_right_action_btn.onclick = function()
{
    getJSON(window.location.href + "push_right", ()=>{});
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
    getJSON(window.location.href + "update_info", ()=>{});
}

reset_task_btn.onclick = function()
{
    console.log("reset task");
    getJSON(window.location.href + "eurobot_task_reset", ()=>{});
}

start_task_purple_btn.onclick = function()
{
    console.log("task purple");
    getJSON(window.location.href + "eurobot_start_purple", ()=>{});
}

start_task_yellow_btn.onclick = function()
{
    console.log("task yellow");
    getJSON(window.location.href + "eurobot_start_yellow", ()=>{});
}

eurobot_kill_task_btn.onclick = function()
{
    console.log("kill task");
    getJSON(window.location.href + "eurobot_kill_task", ()=>{});
}

