import json, csv, os
from bottle import request, route, run, template   

@route('/')
def main():
    name = request.cookies.username or 'Guest'
    return template('charts.tpl')

if __name__ == '__main__':
    run(host='0.0.0.0', port=8080, debug=True)
    print("Server Closed!")