from flask import Flask, request, jsonify,render_template

app = Flask(__name__)

@app.route('/configs', methods=['GET','POST'])
def configs():
    if request.method == 'GET':
        return jsonify({'channel': '11',"default_dvr":"false"}) 
    if request.method == 'POST':
        return jsonify(request.get_json())

@app.route('/file_list', methods=['GET'])
def get_file_list():
    if request.method == 'GET':
        result = [
            {"name":"file1","size":"1024"},
            {"name":"file2","size":"2048"}, 
            {"name":"file3","size":"2048"}, 
            {"name":"file4","size":"2048"}, 
        ]
        return jsonify(result)

@app.route('/delete', methods=['POST'])
def delete():
    if request.method == 'POST':
        return jsonify(request.get_json())


@app.route('/')
def index():
    with open("index.html") as f:
        return f.read()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)