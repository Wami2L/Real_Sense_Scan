import os
import glob
import markdown
from xhtml2pdf import pisa

CSS_STYLE = """
@page {
    size: a4 portrait;
    margin: 2cm 1.8cm 2cm 1.8cm;
    @bottom-right {
        content: counter(page);
        font-family: Helvetica, Arial, sans-serif;
        font-size: 9pt;
        color: #7f8c8d;
    }
    @bottom-left {
        content: "RealSenseScans · Planes Técnicos";
        font-family: Helvetica, Arial, sans-serif;
        font-size: 9pt;
        color: #7f8c8d;
    }
}

body {
    font-family: Helvetica, Arial, sans-serif;
    font-size: 10.5pt;
    line-height: 1.55;
    color: #2c3e50;
}

h1 {
    font-size: 20pt;
    color: #1a365d;
    border-bottom: 2.5px solid #2b6cb0;
    padding-bottom: 8px;
    margin-top: 0;
    margin-bottom: 18px;
}

h2 {
    font-size: 14pt;
    color: #2b6cb0;
    border-bottom: 1px solid #e2e8f0;
    padding-bottom: 5px;
    margin-top: 22px;
    margin-bottom: 12px;
}

h3 {
    font-size: 11.5pt;
    color: #2d3748;
    margin-top: 14px;
    margin-bottom: 6px;
}

p {
    margin-bottom: 10px;
}

ul, ol {
    margin-top: 4px;
    margin-bottom: 12px;
    padding-left: 22px;
}

li {
    margin-bottom: 6px;
}

code {
    background-color: #edf2f7;
    color: #c53030;
    font-family: "Courier New", Courier, monospace;
    font-size: 9.5pt;
    padding: 1px 4px;
    border-radius: 3px;
}

pre {
    background-color: #f7fafc;
    border: 1px solid #e2e8f0;
    border-left: 4px solid #3182ce;
    padding: 10px 14px;
    border-radius: 4px;
    font-size: 9pt;
    line-height: 1.4;
    margin-bottom: 14px;
}

pre code {
    color: #2d3748;
    background-color: transparent;
    padding: 0;
    border: none;
}

blockquote {
    border-left: 3px solid #3182ce;
    background-color: #ebf8ff;
    padding: 8px 12px;
    margin: 12px 0;
    color: #2b6cb0;
}

hr {
    border: none;
    border-top: 1px solid #e2e8f0;
    margin: 20px 0;
}

table {
    width: 100%;
    border-collapse: collapse;
    margin: 14px 0;
}

th, td {
    border: 1px solid #cbd5e0;
    padding: 6px 10px;
    font-size: 9.5pt;
    text-align: left;
}

th {
    background-color: #edf2f7;
    color: #2d3748;
}
"""

def md_to_pdf(md_path, pdf_path):
    print(f"Leyendo {os.path.basename(md_path)}...")
    with open(md_path, "r", encoding="utf-8") as f:
        md_text = f.read()

    html_body = markdown.markdown(md_text, extensions=["extra", "tables"])

    full_html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <style>{CSS_STYLE}</style>
</head>
<body>
{html_body}
</body>
</html>"""

    print(f"Generando {os.path.basename(pdf_path)}...")
    try:
        with open(pdf_path, "wb") as f_out:
            status = pisa.CreatePDF(full_html, dest=f_out, encoding="utf-8")
        
        if status.err:
            print(f"  [ERROR] Falló la conversión de {md_path}: {status.err}")
            return False
        else:
            print(f"  [OK] Creado: {pdf_path}")
            return True
    except PermissionError:
        print(f"  [AVISO] {os.path.basename(pdf_path)} está abierto en otro programa. Ciérralo si deseas regenerarlo.")
        return False

if __name__ == "__main__":
    planes_dir = os.path.dirname(os.path.abspath(__file__))
    md_files = glob.glob(os.path.join(planes_dir, "*.md"))
    
    for md_file in sorted(md_files):
        pdf_file = os.path.splitext(md_file)[0] + ".pdf"
        md_to_pdf(md_file, pdf_file)
    
    print("\nProceso terminado.")
