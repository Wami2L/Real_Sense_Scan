import os
import sys
import markdown
from xhtml2pdf import pisa

def generar_pdf():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    md_path = os.path.join(base_dir, "Registro_Cambios_Live3D.md")
    pdf_path = os.path.join(base_dir, "Registro_Cambios_Live3D.pdf")

    if not os.path.exists(md_path):
        print(f"Error: no se encuentra {md_path}")
        return False

    with open(md_path, "r", encoding="utf-8") as f:
        md_text = f.read()

    html_body = markdown.markdown(
        md_text,
        extensions=["extra", "tables", "codehilite", "nl2br"]
    )

    html_content = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Registro de Cambios - RealSense Live 3D</title>
    <style>
        @page {{
            size: a4 portrait;
            margin: 2cm 1.8cm 2cm 1.8cm;
            @bottom-right {{
                content: "Página " counter(page) " de " counter(pages);
                font-size: 8pt;
                color: #718096;
                font-family: Helvetica, Arial, sans-serif;
            }}
            @bottom-left {{
                content: "RealSense Live 3D - Registro de Cambios";
                font-size: 8pt;
                color: #718096;
                font-family: Helvetica, Arial, sans-serif;
            }}
        }}
        body {{
            font-family: Helvetica, Arial, sans-serif;
            font-size: 10pt;
            line-height: 1.55;
            color: #2d3748;
        }}
        h1 {{
            color: #1a365d;
            font-size: 19pt;
            font-weight: bold;
            border-bottom: 2px solid #3182ce;
            padding-bottom: 6px;
            margin-top: 0;
            margin-bottom: 12px;
        }}
        h2 {{
            color: #2b6cb0;
            font-size: 13pt;
            font-weight: bold;
            border-bottom: 1px solid #e2e8f0;
            padding-bottom: 4px;
            margin-top: 22px;
            margin-bottom: 10px;
        }}
        h3 {{
            color: #2d3748;
            font-size: 11pt;
            font-weight: bold;
            margin-top: 14px;
            margin-bottom: 6px;
        }}
        p {{
            margin-top: 4px;
            margin-bottom: 8px;
        }}
        strong {{
            color: #1a202c;
        }}
        ul, ol {{
            margin-top: 4px;
            margin-bottom: 10px;
            padding-left: 20px;
        }}
        li {{
            margin-bottom: 4px;
        }}
        code {{
            background-color: #edf2f7;
            color: #c53030;
            padding: 1px 4px;
            border-radius: 3px;
            font-family: "Courier New", Courier, monospace;
            font-size: 9pt;
        }}
        pre {{
            background-color: #f7fafc;
            border: 1px solid #e2e8f0;
            border-left: 3px solid #3182ce;
            padding: 8px 12px;
            border-radius: 4px;
            font-family: "Courier New", Courier, monospace;
            font-size: 8.5pt;
            line-height: 1.4;
            margin-top: 6px;
            margin-bottom: 10px;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin-top: 10px;
            margin-bottom: 14px;
            font-size: 8.5pt;
        }}
        th {{
            background-color: #ebf8ff;
            color: #2b6cb0;
            font-weight: bold;
            text-align: left;
            padding: 6px 8px;
            border: 1px solid #cbd5e0;
        }}
        td {{
            padding: 5px 8px;
            border: 1px solid #e2e8f0;
            vertical-align: top;
        }}
        blockquote {{
            background-color: #fffaf0;
            border-left: 3px solid #dd6b20;
            padding: 8px 12px;
            margin: 10px 0;
            color: #7b341e;
            font-size: 9pt;
        }}
        hr {{
            border: 0;
            border-top: 1px solid #e2e8f0;
            margin: 16px 0;
        }}
        kbd {{
            background-color: #edf2f7;
            border: 1px solid #cbd5e0;
            border-radius: 3px;
            padding: 1px 4px;
            font-size: 8pt;
            font-family: "Courier New", Courier, monospace;
        }}
    </style>
</head>
<body>
    {html_body}
</body>
</html>"""

    print(f"Compilando PDF desde {md_path}...")
    with open(pdf_path, "wb") as f_out:
        status = pisa.CreatePDF(html_content, dest=f_out, encoding="utf-8")

    if status.err:
        print(f"Error al generar el PDF: {status.err}")
        return False
    else:
        size_kb = os.path.getsize(pdf_path) / 1024
        print(f"PDF generado con éxito: {pdf_path} ({size_kb:.1f} KB)")
        return True

if __name__ == "__main__":
    exito = generar_pdf()
    sys.exit(0 if exito else 1)
