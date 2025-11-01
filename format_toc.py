import re

def format_toc(content):
    lines = content.split('\n')
    toc_start_index = -1
    toc_end_index = -1
    in_toc = False

    # Find the start and end of the Table of Contents section
    # The TOC starts right after the "Table of Contents" line.
    # It ends just before the line with the form feed character.
    for i, line in enumerate(lines):
        if line.strip() == "Table of Contents":
            toc_start_index = i
            in_toc = True
        elif in_toc and '\f' in line:
            toc_end_index = i
            break

    if toc_start_index == -1 or toc_end_index == -1:
        print("Could not find the TOC section using the expected markers.")
        return content

    toc_lines = lines[toc_start_index + 1 : toc_end_index]

    new_toc = ["**Table of Contents**\n"]

    for line in toc_lines:
        line = line.strip()
        if not line:
            continue

        # Use regex to strip dots and page numbers from the end
        match = re.match(r'^(.*?)\s*\.{2,}\s*(\d+)$', line)
        if match:
            title = match.group(1).strip()
        else:
            title = line.strip()

        # Determine indentation level based on section numbering (e.g., "1.", "1.2.", "1.2.3.")
        indent_level = 0
        section_match = re.match(r'^([\d\.]+|[A-Z]\.)\s', title)
        if section_match:
            # Count dots to determine nesting level for numbered sections
            section_part = section_match.group(1)
            indent_level = section_part.count('.')
            # Handle appendices like "A." which have 1 dot but are level 0
            if '.' in section_part and section_part.replace('.', '').isalpha():
                indent_level = 0

        # Handle top-level un-numbered sections
        if title.startswith("Chapter") or title.startswith("Appendix") or title in ["Colophon", "IP Contributors", "Legal Disclaimer Notice"]:
            indent_level = 0

        new_toc.append("  " * indent_level + f"* {title}")

    # Reconstruct the file content with the new, formatted TOC
    # This replaces the old TOC section entirely.
    new_content_lines = lines[:toc_start_index] + new_toc + lines[toc_end_index:]
    return '\n'.join(new_content_lines)


def main():
    filepath = 'datasheets/RP2040/RP2040-Datasheet.md'
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            original_content = f.read()

        formatted_content = format_toc(original_content)

        if original_content != formatted_content:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(formatted_content)
            print("TOC formatted successfully.")
        else:
            print("TOC already formatted or markers not found; no changes made.")

    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
