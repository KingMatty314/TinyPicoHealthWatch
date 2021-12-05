import json, csv, os
from bottle import request, route, run, template   

@route('/')
def main():
    name = request.cookies.username or 'Guest'
    path = os.path.join(os.getcwd(), 'views/charts.tpl')
    return template(path)

if __name__ == '__main__':
    run(host='0.0.0.0', port=8080, debug=True)
    print("Server Closed!")