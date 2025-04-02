from flask import Flask, request, render_template_string
import pandas as pd
import folium
import io
import os

app = Flask(__name__)

# HTML template for the upload form and results
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>CSV Data Viewer</title>
    <style>
        table {
            border-collapse: collapse;
            width: 80%;
            margin: 20px auto;
        }
        th, td {
            border: 1px solid black;
            padding: 8px;
            text-align: center;
        }
        th {
            background-color: #f2f2f2;
        }
    </style>
</head>
<body>
    <h1>Upload CSV File</h1>
    <form method="POST" enctype="multipart/form-data" action="/">
        <input type="file" name="csv_file" accept=".csv">
        <input type="submit" value="Upload">
    </form>
    {% if table_html %}
        <h2>Data Table</h2>
        {{ table_html | safe }}
        <h2>Map</h2>
        <iframe src="/map" width="80%" height="500" style="border:none; margin: 20px auto; display: block;"></iframe>
    {% endif %}
</body>
</html>
"""

@app.route('/', methods=['GET', 'POST'])
def upload_file():
    table_html = None
    if request.method == 'POST':
        # Check if a file was uploaded
        if 'csv_file' not in request.files:
            return "No file uploaded", 400
        file = request.files['csv_file']
        if file.filename == '':
            return "No file selected", 400
        
        # Read the CSV file into a pandas DataFrame
        try:
            df = pd.read_csv(file)
            required_columns = ['lat', 'lon', 'alt', 'v_lat', 'v_lon', 'v_alt']
            if not all(col in df.columns for col in required_columns):
                return "CSV must contain lat, lon, alt, v_lat, v_lon, v_alt columns", 400
            
            # Generate HTML table
            table_html = df.to_html(index=False, classes='table')
            
            # Generate map
            m = folium.Map(location=[df['lat'].mean(), df['lon'].mean()], zoom_start=10)
            for _, row in df.iterrows():
                folium.Marker(
                    location=[row['lat'], row['lon']],
                    popup=f"Lat: {row['lat']}<br>Lon: {row['lon']}<br>Alt: {row['alt']}<br>"
                          f"V_Lat: {row['v_lat']}<br>V_Lon: {row['v_lon']}<br>V_Alt: {row['v_alt']}"
                ).add_to(m)
            m.save('static/map.html')
        
        except Exception as e:
            return f"Error processing CSV: {str(e)}", 400
    
    return render_template_string(HTML_TEMPLATE, table_html=table_html)

@app.route('/map')
def serve_map():
    return app.send_static_file('map.html')

if __name__ == '__main__':
    # Create static directory if it doesn't exist
    if not os.path.exists('static'):
        os.makedirs('static')
    app.run(debug=True, host='0.0.0.0', port=5000)