import os, sys
sys.path.insert(0,os.path.dirname(os.path.realpath(__file__)))

import time
from server_config import *

def __log(status="INFO", message=""):
    print("[{status}] {message}".format(status=status, message=message))

    ## logging
    # app.logger.debug("Logging debug messages")
    # app.logger.warning("Logging warnings")
    # app.logger.error("Logging erros")

@app.route("/")
def index():
    return render_template("index.html", logged_in=False)

@app.route("/welcome")
def header():
    return render_template("includes/header.html", logged_in=False)

@app.route("/test")
def test():
    return "Test Successful"

@app.route("/login", methods=["GET", "POST"])
def login():
    error = None

    uname = request.form["username"]
    key   = request.form["password"]

    # if uname in session:
    #     return "Logged in as {uname}".format(uname=uname)

    # session["username"] = uname

    if request.method == "POST":
        return "LOGIN VIA POST"
    else:
        return "LOGIN VIA GET"

@app.route("/logout")
def logout():
    session.pop("username", None)
    return redirect(url_for("index"))

@app.route("/shutdown")
@app.route("/kill_server")
def shutdown_server():
    __log("Shutting down the server ...")

    try:
        thread.exit()
    except:
        __log("Failed to exit server thread ...")

    sys.exit(0)