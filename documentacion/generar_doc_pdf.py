import os
import subprocess
import sys

def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])

try:
    import markdown
except ImportError:
    install('markdown')
    import markdown

try:
    from xhtml2pdf import pisa
except ImportError:
    install('xhtml2pdf')
    from xhtml2pdf import pisa

def convert_md_to_pdf(md_file, pdf_file):
    print(f"Leyendo {md_file}...")
    with open(md_file, 'r', encoding='utf-8') as f:
        md_text = f.read()

    print("Convirtiendo Markdown a HTML...")
    html_body = markdown.markdown(md_text, extensions=['extra'])

    html_content = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <style>
            body {{
                font-family: Helvetica, Arial, sans-serif;
                font-size: 11pt;
                line-height: 1.5;
                color: #333;
                margin: 20px;
            }}
            h1, h2, h3 {{
                color: #2c3e50;
            }}
            h1 {{ border-bottom: 2px solid #3498db; padding-bottom: 5px; }}
            h2 {{ margin-top: 25px; border-bottom: 1px solid #ddd; padding-bottom: 5px; }}
            code {{ background-color: #f8f9fa; padding: 2px 4px; border-radius: 3px; font-family: "Courier New", Courier, monospace; font-size: 10pt; }}
            pre {{ background-color: #f8f9fa; padding: 10px; border-radius: 5px; border: 1px solid #ddd; }}
            li {{ margin-bottom: 5px; }}
        </style>
    </head>
    <body>
        {html_body}
    </body>
    </html>
    """

    print(f"Generando {pdf_file}...")
    with open(pdf_file, "w+b") as result_file:
        pisa_status = pisa.CreatePDF(
            html_content,                
            dest=result_file)
            
    if pisa_status.err:
        print(f"Hubo errores en la conversión: {pisa_status.err}")
    else:
        print(f"PDF generado exitosamente en {pdf_file}")

if __name__ == "__main__":
    convert_md_to_pdf("Documentacion_RealSenseScans.md", "Documentacion_RealSenseScans.pdf")
