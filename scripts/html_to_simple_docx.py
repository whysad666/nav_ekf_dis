#!/usr/bin/env python3
"""Convert the limited report HTML used in this workspace to a simple DOCX."""

from __future__ import annotations

import argparse
import zipfile
from pathlib import Path

from lxml import etree, html


W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
R_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
XML_NS = "http://www.w3.org/XML/1998/namespace"
NSMAP = {"w": W_NS, "r": R_NS}


def w(tag: str) -> str:
    return f"{{{W_NS}}}{tag}"


def add_run(paragraph, text: str, *, bold=False, superscript=False, subscript=False):
    if not text:
        return
    run = etree.SubElement(paragraph, w("r"))
    props = etree.SubElement(run, w("rPr"))
    if bold:
        etree.SubElement(props, w("b"))
    if superscript or subscript:
        valign = etree.SubElement(props, w("vertAlign"))
        valign.set(w("val"), "superscript" if superscript else "subscript")
    text_node = etree.SubElement(run, w("t"))
    if text[:1].isspace() or text[-1:].isspace():
        text_node.set(f"{{{XML_NS}}}space", "preserve")
    text_node.text = text


def add_inline_content(paragraph, node, *, bold=False, superscript=False, subscript=False):
    add_run(paragraph, node.text or "", bold=bold, superscript=superscript, subscript=subscript)
    for child in node:
        tag = child.tag.lower() if isinstance(child.tag, str) else ""
        add_inline_content(
            paragraph,
            child,
            bold=bold or tag in {"b", "strong"},
            superscript=superscript or tag == "sup",
            subscript=subscript or tag == "sub",
        )
        add_run(paragraph, child.tail or "", bold=bold, superscript=superscript, subscript=subscript)


def add_paragraph(body, node, style=None, centered=False):
    paragraph = etree.SubElement(body, w("p"))
    props = etree.SubElement(paragraph, w("pPr"))
    if style:
        style_node = etree.SubElement(props, w("pStyle"))
        style_node.set(w("val"), style)
    if centered:
        align = etree.SubElement(props, w("jc"))
        align.set(w("val"), "center")
    add_inline_content(paragraph, node)
    return paragraph


def set_cell_text(cell, source_cell, *, bold=False):
    paragraph = etree.SubElement(cell, w("p"))
    props = etree.SubElement(paragraph, w("pPr"))
    align = etree.SubElement(props, w("jc"))
    align.set(w("val"), "center")
    add_inline_content(paragraph, source_cell, bold=bold)


def add_table(body, source_table):
    table = etree.SubElement(body, w("tbl"))
    props = etree.SubElement(table, w("tblPr"))
    style = etree.SubElement(props, w("tblStyle"))
    style.set(w("val"), "TableGrid")
    width = etree.SubElement(props, w("tblW"))
    width.set(w("w"), "0")
    width.set(w("type"), "auto")
    for source_row in source_table.xpath(".//tr"):
        row = etree.SubElement(table, w("tr"))
        for source_cell in source_row.xpath("./th|./td"):
            cell = etree.SubElement(row, w("tc"))
            cell_props = etree.SubElement(cell, w("tcPr"))
            cell_width = etree.SubElement(cell_props, w("tcW"))
            cell_width.set(w("w"), "0")
            cell_width.set(w("type"), "auto")
            set_cell_text(cell, source_cell, bold=source_cell.tag.lower() == "th")


def document_xml(source_html: Path) -> bytes:
    source = html.parse(str(source_html)).getroot()
    document = etree.Element(w("document"), nsmap=NSMAP)
    body = etree.SubElement(document, w("body"))
    for node in source.xpath("//body/*"):
        tag = node.tag.lower() if isinstance(node.tag, str) else ""
        classes = set((node.get("class") or "").split())
        if tag == "h1":
            add_paragraph(body, node, style="Title", centered=True)
        elif tag == "h2":
            add_paragraph(body, node, style="Heading1")
        elif tag == "p":
            add_paragraph(body, node, centered="equation" in classes)
        elif tag == "table":
            add_table(body, node)

    section = etree.SubElement(body, w("sectPr"))
    page_size = etree.SubElement(section, w("pgSz"))
    page_size.set(w("w"), "11906")
    page_size.set(w("h"), "16838")
    margins = etree.SubElement(section, w("pgMar"))
    margins.set(w("top"), "1247")
    margins.set(w("right"), "1304")
    margins.set(w("bottom"), "1247")
    margins.set(w("left"), "1304")
    margins.set(w("header"), "708")
    margins.set(w("footer"), "708")
    margins.set(w("gutter"), "0")
    return etree.tostring(document, xml_declaration=True, encoding="UTF-8", standalone="yes")


def styles_xml() -> bytes:
    styles = etree.Element(w("styles"), nsmap={"w": W_NS})

    normal = etree.SubElement(styles, w("style"))
    normal.set(w("type"), "paragraph")
    normal.set(w("default"), "1")
    normal.set(w("styleId"), "Normal")
    etree.SubElement(normal, w("name")).set(w("val"), "Normal")
    normal_p = etree.SubElement(normal, w("pPr"))
    spacing = etree.SubElement(normal_p, w("spacing"))
    spacing.set(w("line"), "360")
    spacing.set(w("lineRule"), "auto")
    spacing.set(w("after"), "100")
    normal_r = etree.SubElement(normal, w("rPr"))
    fonts = etree.SubElement(normal_r, w("rFonts"))
    for key in ("ascii", "hAnsi"):
        fonts.set(w(key), "Liberation Serif")
    fonts.set(w("eastAsia"), "Noto Serif CJK SC")
    size = etree.SubElement(normal_r, w("sz"))
    size.set(w("val"), "22")
    etree.SubElement(normal_r, w("szCs")).set(w("val"), "22")

    for style_id, name, size_value in (("Title", "Title", "36"), ("Heading1", "heading 1", "28")):
        style = etree.SubElement(styles, w("style"))
        style.set(w("type"), "paragraph")
        style.set(w("styleId"), style_id)
        etree.SubElement(style, w("name")).set(w("val"), name)
        etree.SubElement(style, w("basedOn")).set(w("val"), "Normal")
        pprops = etree.SubElement(style, w("pPr"))
        spacing = etree.SubElement(pprops, w("spacing"))
        spacing.set(w("before"), "180")
        spacing.set(w("after"), "100")
        rprops = etree.SubElement(style, w("rPr"))
        etree.SubElement(rprops, w("b"))
        etree.SubElement(rprops, w("sz")).set(w("val"), size_value)
        etree.SubElement(rprops, w("szCs")).set(w("val"), size_value)

    table = etree.SubElement(styles, w("style"))
    table.set(w("type"), "table")
    table.set(w("styleId"), "TableGrid")
    etree.SubElement(table, w("name")).set(w("val"), "Table Grid")
    borders = etree.SubElement(etree.SubElement(table, w("tblPr")), w("tblBorders"))
    for side in ("top", "left", "bottom", "right", "insideH", "insideV"):
        border = etree.SubElement(borders, w(side))
        border.set(w("val"), "single")
        border.set(w("sz"), "4")
        border.set(w("color"), "666666")
    return etree.tostring(styles, xml_declaration=True, encoding="UTF-8", standalone="yes")


CONTENT_TYPES = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
  <Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
</Types>
"""

PACKAGE_RELS = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
</Relationships>
"""

DOCUMENT_RELS = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"/>
"""


def build_docx(source_html: Path, output_docx: Path) -> None:
    output_docx.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output_docx, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", CONTENT_TYPES)
        archive.writestr("_rels/.rels", PACKAGE_RELS)
        archive.writestr("word/document.xml", document_xml(source_html))
        archive.writestr("word/styles.xml", styles_xml())
        archive.writestr("word/_rels/document.xml.rels", DOCUMENT_RELS)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_html", type=Path)
    parser.add_argument("output_docx", type=Path)
    args = parser.parse_args()
    build_docx(args.source_html, args.output_docx)


if __name__ == "__main__":
    main()
