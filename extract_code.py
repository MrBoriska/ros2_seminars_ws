import docx

doc = docx.Document('/tmp/file_attachments/илья зыбин, модуль 2/[ROS2-2026] Модуль 2 - Планирование - Вариант №10 - Зыбин Илья.docx')
printing = False
for para in doc.paragraphs:
    if 'Листинг 1' in para.text:
        printing = True
    if printing:
        print(para.text)
