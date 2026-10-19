import os
import subprocess
import sys
import glob

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

    print(f"Convirtiendo {md_file} a HTML...")
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
                line-height: 1.6;
                color: #2b2b2b;
                margin: 30px;
            }}
            h1, h2, h3 {{
                color: #2c3e50;
            }}
            h1 {{ border-bottom: 2px solid #3498db; padding-bottom: 10px; font-size: 18pt; margin-bottom: 20px; }}
            h2 {{ margin-top: 30px; border-bottom: 1px solid #ddd; padding-bottom: 5px; font-size: 14pt; }}
            code {{ background-color: #f4f5f6; padding: 2px 4px; border-radius: 4px; font-family: "Courier New", Courier, monospace; font-size: 10pt; color: #c0392b; }}
            pre {{ background-color: #f4f5f6; padding: 12px; border-radius: 6px; border: 1px solid #e1e4e8; margin-bottom: 15px; }}
            pre code {{ color: #24292e; background-color: transparent; border: none; padding: 0; }}
            li {{ margin-bottom: 8px; }}
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
        print(f"PDF generado exitosamente: {pdf_file}")

if __name__ == "__main__":
    md_files = glob.glob("*.md")
    for md_file in md_files:
        pdf_file = md_file.replace(".md", ".pdf")
        convert_md_to_pdf(md_file, pdf_file)
    
    print("\n¡Todos los documentos han sido convertidos a PDF con éxito!")
