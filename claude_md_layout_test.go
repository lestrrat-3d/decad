package decad_test

import (
	"fmt"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"testing"
	"unicode/utf8"

	"github.com/stretchr/testify/require"
)

// Regression guard for CLAUDE.md's Layout table (see CLAUDE.md's own
// "## Layout" section and ~/.claude/docs/agent-instructions.md's "prefer
// mechanical doc/code checks over exhortations"): each feature PR tends to
// append prose to a row's description cell instead of pointing elsewhere, so
// this test fails the moment the file grows past its budget, a row's cell
// grows past its own, or a row stops parsing as a row at all. A mechanical
// comparison that can FAIL beats a comment asking authors to "keep it short".
//
// The guard is only as strong as the parser's reach, so parseCLAUDEMD is a
// TOTAL scanner: every line of the file passes through one classifier exactly
// once, and a line the classifier reads as a heading or as part of a table
// without being the one declared spelling is REFUSED wherever it sits — not
// skipped, and not only inside the section. Its own doc comment states the
// classes and the rules derived from them.
//
// Two deliberate policies bound what is left unmeasured, and neither is a bug:
//
//   - Prose is capped only by the whole-file byte budget. The Layout section's
//     own lead paragraph, every paragraph outside the section, and the text
//     inside a fenced or indented code block all carry as many characters as
//     the budget leaves. The per-cell cap governs TABLE CELLS, which is where
//     the detail an author is tempted to inline actually lands.
//   - layoutCellMaxChars counts RUNES, not bytes, so a 300-rune CJK cell is
//     900 bytes. The byte budget is the backstop for that too.
const (
	claudeMDPath = "CLAUDE.md"
	// claudeMDMaxBytes is the whole-file budget. Raising it is a policy
	// change, not a bug fix — trim a Layout row instead.
	claudeMDMaxBytes = 32768
	// layoutCellMaxChars caps a Layout row's description (second) column.
	// A row that needs more detail belongs in the file/doc it names, with
	// this cell trimmed back to a short pointer.
	//
	// The cap binds real rows, and a SUBSECTION number is among the first
	// details it pushes back out. The `loft_build.go` row is the worked
	// case: naming docs/loft-design.md §5.1 and §5.2 beside the §5 that row
	// already points at costs 12 runes, which its 292-rune description cell
	// affords only because the row carries no wording its pointer can do
	// without — 8 runes of headroom is all that is left, so the next
	// parenthetical appended to it fails the check below. A row that cannot
	// pay for a subsection number that way names the SECTION and lets the
	// section carry its own subsections; where a subsection's content
	// matters to the reader, the row spells that content in words instead
	// of by number, which is what the `loft_build.go` row also does for
	// §5.2's placement re-lift and its `delta`.
	layoutCellMaxChars = 300
	// designDocDir and designDocSuffix spell CLAUDE.md's own Conventions
	// rule, "Design docs live in `docs/<topic>-design.md`", once. Every file
	// matching them must carry a Layout row, the mirror of the root-".go"
	// coverage check: a design doc with no row is a doc CLAUDE.md never
	// points at. coverageTargets owns what keeps either check from matching
	// nothing and covering nothing while reporting PASS.
	designDocDir    = "docs"
	designDocSuffix = "-design.md"
)

// pathCell matches a Layout row's first column, which holds one of: a
// single file ("`doc.go`"), two files joined by " / "
// ("`recipe.go` / `recipe_wire.go`"), a directory ("`examples/`",
// "`.github/workflows/`"), or a docs file ("`docs/api-design.md`") — one or
// two backtick-quoted tokens, never containing "|".
const pathCell = "`[^`]+`(?:\\s*/\\s*`[^`]+`)?"

// layoutRowRe anchors on that known, "|"-free column-1 shape to find the
// true column boundary, rather than naively splitting the line on every
// "|". Several rows use a literal "|" in their description as absolute-value
// bars in prose (e.g. "`|area|`", "\|estimate-true\|"), which a naive split
// would mistake for extra columns. The trailing "\|$" anchor takes the
// line's OWN final "|" as the closing delimiter, so everything between the
// two matched delimiters — however many literal pipes it holds — is column
// 2 intact.
var layoutRowRe = regexp.MustCompile(`^\|\s*(` + pathCell + `)\s*\|(.*)\|$`)

var backtickPathRe = regexp.MustCompile("`([^`]+)`")

// layoutSeparatorRe matches the "|---|---|" alignment row a declared table
// header must be followed by (optional padding and alignment colons allowed).
// It is the only line the scanner skips as scaffolding: every OTHER line the
// classifier reads as part of a table is a data row, and one that fails
// layoutRowRe is malformed rather than scaffolding.
var layoutSeparatorRe = regexp.MustCompile(`^\|(?:\s*:?-+:?\s*\|)+$`)

// layoutHeading is the one "##" heading CLAUDE.md's Layout tables live under.
// The file may carry exactly one of it, spelled exactly this way: parseCLAUDEMD
// binds its whole parse to that heading, so a second copy anywhere in the file
// — before the real one as a decoy, after it as an annex, fenced off so it
// renders invisibly — would silently decide which rows are measured and which
// are never seen.
const layoutHeading = "## Layout"

// layoutHeadingLookalikeRe matches every line a reader would take for the
// Layout heading: leading hashes, any run of whitespace (ASCII or Unicode, so
// a no-break space is caught), the word "layout" in any case, an optional
// closing hash sequence. Exactly one spelling of it — layoutHeading itself —
// is the heading; every other match is REFUSED, because each renders as a
// Layout section the scanner would not bind to, leaving its rows unmeasured.
var layoutHeadingLookalikeRe = regexp.MustCompile(`(?i)^[ \t]*#{1,6}[\s\p{Zs}]*layout[\s\p{Zs}]*#*[\s\p{Zs}]*$`)

// layoutTableHeader and layoutTableSeparator are the two scaffolding lines a
// Layout table opens with, spelled once so the refusals, the fixtures and
// layoutSeparatorRe above cannot drift apart.
const (
	layoutTableHeader    = "| Path | Responsibility |"
	layoutTableSeparator = "|---|---|"
)

// nonLayoutTableHeaders declares every table CLAUDE.md carries OUTSIDE its
// Layout section, spelled exactly, beside the heading it belongs under. The
// scanner refuses every other table shape anywhere in the file, so adding a
// table to CLAUDE.md means declaring it here first. That refuse-by-default is
// the point: a table the guard does not know about holds cells nothing
// measures. Declared rows are still cell-capped — see "every table cell is
// within the cell budget" — so declaring a table bounds it rather than
// exempting it.
var nonLayoutTableHeaders = []struct{ header, section string }{
	{"| Before writing | Read |", "## Read before you write"},
}

var (
	// atxHeadingRe is CommonMark's ATX heading: up to three spaces of
	// indent, one to six hashes, then a space or the end of the line. A
	// fourth space of indent makes the line code or a continuation, not a
	// heading, which is why this regex — and not a TrimSpace — decides.
	atxHeadingRe = regexp.MustCompile(`^ {0,3}(#{1,6})(?:[ \t].*)?$`)
	// setextRuleRe matches a line of only "=" or "-". Under a paragraph line
	// that is a setext heading; after a blank it is a thematic break.
	setextRuleRe = regexp.MustCompile(`^ {0,3}(?:=+|-+)[ \t]*$`)
	// fenceRe matches a code-fence delimiter, whose run length and character
	// the scanner carries across lines so it knows which line closes it.
	fenceRe = regexp.MustCompile("^ {0,3}(`{3,}|~{3,})(.*)$")
	// htmlBlockRe matches a line that opens an HTML block. The scanner reads
	// Markdown structure only, so an HTML table or comment is text no row
	// check measures.
	htmlBlockRe = regexp.MustCompile(`^ {0,3}<[a-zA-Z!/?]`)
)

// The fixtures below share three strings: the invented sub-heading every
// fixture section opens its table under, and the two refusal fragments the
// table and heading branches are pinned by.
const (
	inventedSubHeading    = "### Invented sub-table"
	wantStrayTable        = "outside any declared table"
	wantMisspelledHeading = "not spelled that way"
)

// layoutRow is one parsed data row of CLAUDE.md's "## Layout" table.
type layoutRow struct {
	line        int      // 1-based source line, for failure messages
	paths       []string // one or two paths named in column 1
	description string   // raw column 2 text
}

// tableCell is one cell of one declared non-Layout table row, kept so the
// cell budget covers every table in the file rather than the Layout ones only.
type tableCell struct {
	line   int
	header string
	column int
	text   string
}

// claudeMDTables is everything parseCLAUDEMD measured.
type claudeMDTables struct {
	rows       []layoutRow // data rows of the Layout tables
	otherCells []tableCell // cells of every declared non-Layout table
}

// fenceDelim reports whether the line is a code-fence delimiter, returning the
// delimiter run and the info string that follows it. A backtick fence's info
// string may not itself hold a backtick, which is what keeps an ordinary
// sentence full of code spans from being read as a fence.
func fenceDelim(raw string) (delim, info string, ok bool) {
	m := fenceRe.FindStringSubmatch(raw)
	if m == nil {
		return "", "", false
	}
	if m[1][0] == '`' && strings.Contains(m[2], "`") {
		return "", "", false
	}
	return m[1], m[2], true
}

// tablePipeIndices returns the byte offsets of every "|" the Markdown table
// syntax would read as a cell delimiter: one that is neither backslash-escaped
// nor inside a backtick code span. Prose holds both other spellings — several
// Layout rows write absolute-value bars as `|area|` or \|estimate-true\| — so
// a plain strings.Contains would read ordinary sentences as table rows.
// Backslash, backtick and "|" are all ASCII, so scanning bytes cannot land
// inside a multi-byte rune.
func tablePipeIndices(s string) []int {
	var out []int
	for i := 0; i < len(s); i++ {
		switch s[i] {
		case '\\':
			i++ // the next byte is escaped, "|" included
		case '`':
			n := 1
			for i+n < len(s) && s[i+n] == '`' {
				n++
			}
			closed := -1
			for j := i + n; j < len(s); {
				if s[j] != '`' {
					j++
					continue
				}
				m := 1
				for j+m < len(s) && s[j+m] == '`' {
					m++
				}
				if m == n {
					closed = j + m
					break
				}
				j += m
			}
			if closed < 0 {
				i += n - 1 // an unclosed run is literal backticks
			} else {
				i = closed - 1
			}
		case '|':
			out = append(out, i)
		}
	}
	return out
}

// splitRowCells cuts a table row at the delimiters tablePipeIndices found,
// dropping the empty edge cells a leading or trailing "|" produces.
func splitRowCells(s string, pipes []int) []string {
	cells := make([]string, 0, len(pipes)+1)
	prev := 0
	for _, p := range pipes {
		cells = append(cells, s[prev:p])
		prev = p + 1
	}
	cells = append(cells, s[prev:])
	if len(cells) > 0 && strings.TrimSpace(cells[0]) == "" {
		cells = cells[1:]
	}
	if len(cells) > 0 && strings.TrimSpace(cells[len(cells)-1]) == "" {
		cells = cells[:len(cells)-1]
	}
	return cells
}

// declaredNonLayoutHeader returns the heading a declared non-Layout table
// belongs under, or "" if the line is not one of them.
func declaredNonLayoutHeader(trimmed string) string {
	for _, t := range nonLayoutTableHeaders {
		if trimmed == t.header {
			return t.section
		}
	}
	return ""
}

// declaredHeaders renders every table header the file may carry, for the
// refusal that names them.
func declaredHeaders() string {
	out := []string{layoutTableHeader}
	for _, t := range nonLayoutTableHeaders {
		out = append(out, t.header)
	}
	return strings.Join(out, ", ")
}

// requireSeparator refuses a declared table header whose alignment row does
// not follow it, since without one the lines beneath are not a table at all.
func requireSeparator(lines []string, i int, header string) error {
	if i+1 >= len(lines) || !layoutSeparatorRe.MatchString(strings.TrimSpace(lines[i+1])) {
		return fmt.Errorf("CLAUDE.md:%d: %q header is not followed by its %q separator, so the lines beneath it are not a table at all",
			i+1, header, layoutTableSeparator)
	}
	return nil
}

// parseCLAUDEMD scans the WHOLE file once and extracts every data row of every
// "## Layout" table, plus the cells of every declared non-Layout table.
//
// The checks those rows feed are only as good as the scanner's reach: text the
// scanner never visits carries an unbounded description cell and names whatever
// path it likes with nothing measuring either. So the default is REFUSE, not
// skip. Every line passes through one classifier exactly once, in this order,
// and each class is decided by the format's own rules rather than by a
// TrimSpace that erases indentation the format makes meaningful:
//
//	fence delimiter → fenced content → blank → indented (4+ spaces) →
//	Layout heading → any other ATX heading → setext underline →
//	table line → HTML block → prose
//
// Section and table membership are DERIVED from that classification, under
// five structural rules:
//
//  1. layoutHeading appears exactly once, spelled exactly. A second copy —
//     earlier, later, or inside a code fence — would capture the parse and
//     leave the rows it did not capture unmeasured. A line that READS as the
//     Layout heading in any other spelling (a setext underline, a closing "##"
//     sequence, extra or non-ASCII whitespace, different case) is refused for
//     the same reason: it renders as a Layout section the scanner cannot bind
//     to.
//  2. Every table opens with a declared header — layoutTableHeader inside the
//     Layout section, one of nonLayoutTableHeaders outside it — immediately
//     followed by its separator row. Any other line the table syntax would
//     read as part of a table is refused wherever it sits, whatever its header
//     cells say and whether or not it carries leading pipes.
//  3. Inside the section, the only prose allowed is the section's own lead,
//     ahead of the first table. Once a table has opened, loose prose is
//     refused, since prose beside the rows reads as row text while escaping
//     every per-row check. A sub-heading or a fenced code example is not prose
//     and is allowed.
//  4. Every line of an open table parses as a row: a Layout row through
//     layoutRowRe, whose first column is one or two backtick-quoted paths; a
//     declared non-Layout row through its cell split. A row missing its
//     closing "|", one whose first column is not a path, and one stranded
//     outside any table are all refusals.
//  5. An HTML block is refused anywhere in the file, and a code fence left
//     open at EOF is refused, since either hides text from every rule above.
//
// Content the format renders as code — inside a fence, or indented four spaces
// or more — is skipped rather than classified, which is what lets the section
// hold a fenced example. A fence is the one place the two anchor strings are
// still refused: a fenced copy of layoutHeading or layoutTableHeader is one
// un-fencing edit away from capturing the parse. An over-indented copy is not
// refused, because under the format's own three-space rule it is not the
// canonical spelling at all and can never be read as the anchor.
func parseCLAUDEMD(content string) (*claudeMDTables, error) {
	var (
		out       claudeMDTables
		headings  int    // "## Layout" headings seen so far
		inSection bool   // inside the Layout section
		inTable   bool   // inside an open table's row block
		layoutTbl bool   // that open table is a Layout table
		sawTable  bool   // a table has opened in this section
		tblHeader string // the open table's declared header
		fence     string // the open fence's delimiter run, "" when not fenced
		fenceAt   int    // where that fence opened
		prevProse bool   // the previous line was a paragraph line
	)

	lines := strings.Split(content, "\n")
	for i := 0; i < len(lines); i++ {
		raw := lines[i]
		at := i + 1
		trimmed := strings.TrimSpace(raw)

		// Fence state is the one thing carried across lines: without it the
		// scanner cannot tell a code example from the structure it imitates.
		if fence != "" {
			if d, info, ok := fenceDelim(raw); ok && d[0] == fence[0] && len(d) >= len(fence) && strings.TrimSpace(info) == "" {
				fence = ""
				continue
			}
			switch trimmed {
			case layoutHeading:
				headings++
				if headings > 1 {
					return nil, fmt.Errorf("CLAUDE.md:%d: a second %q heading, this one inside a code fence — the file may carry exactly one, and a fenced copy is one un-fencing edit away from capturing the parse and leaving every row of the other section unmeasured: %q",
						at, layoutHeading, trimmed)
				}
				return nil, fmt.Errorf("CLAUDE.md:%d: a %q heading inside a code fence — the file's one Layout heading may not be a fenced copy, which is one un-fencing edit away from capturing the parse: %q",
					at, layoutHeading, trimmed)
			case layoutTableHeader:
				return nil, fmt.Errorf("CLAUDE.md:%d: a %q table header inside a code fence — it is one un-fencing edit away from opening a table whose rows this guard would have to measure: %q",
					at, layoutTableHeader, trimmed)
			}
			continue
		}
		if d, _, ok := fenceDelim(raw); ok {
			fence, fenceAt = d, at
			inTable, prevProse = false, false
			continue
		}

		if trimmed == "" {
			inTable, prevProse = false, false
			continue
		}

		// Four spaces or a tab of indent: the format reads this as code or as
		// a continuation line, never as a heading or a table. prevProse is
		// left alone, so a continuation still underlines like the paragraph
		// it continues. The one refusal is a row indented out of the table it
		// appears to continue, which the format drops and no row check would
		// then see.
		if strings.HasPrefix(raw, "    ") || strings.HasPrefix(raw, "\t") {
			if inTable && len(tablePipeIndices(raw)) > 0 {
				return nil, fmt.Errorf("CLAUDE.md:%d: a table row indented four spaces or more — that indent ends the table the row appears to continue, leaving it measured by nothing: %q",
					at, trimmed)
			}
			continue
		}

		if layoutHeadingLookalikeRe.MatchString(raw) {
			if raw != layoutHeading {
				return nil, fmt.Errorf("CLAUDE.md:%d: a heading that reads as the %q heading but is not spelled that way — the guard binds to that one spelling, so this section renders as Layout while every row under it goes unmeasured: %q",
					at, layoutHeading, raw)
			}
			headings++
			if headings > 1 {
				return nil, fmt.Errorf("CLAUDE.md:%d: a second %q heading — the file may carry exactly one, since a duplicate captures the parse and leaves every row of the other section unmeasured: %q",
					at, layoutHeading, trimmed)
			}
			inSection, inTable, sawTable, prevProse = true, false, false, false
			continue
		}

		if m := atxHeadingRe.FindStringSubmatch(raw); m != nil {
			// The Layout section groups its rows into several tables under
			// "###" sub-headings, so it ends at the next "#"/"##" heading
			// rather than at the first blank line after a table.
			if len(m[1]) <= 2 {
				inSection, sawTable = false, false
			}
			inTable, prevProse = false, false
			continue
		}

		if setextRuleRe.MatchString(raw) {
			if prevProse {
				return nil, fmt.Errorf("CLAUDE.md:%d: %q underlined with %q is a setext heading — CLAUDE.md's headings are ATX (%q), and a setext one read as prose opens a whole section this guard never measures",
					at, strings.TrimSpace(lines[i-1]), trimmed, layoutHeading)
			}
			inTable, prevProse = false, false // a thematic break
			continue
		}

		if pipes := tablePipeIndices(raw); len(pipes) > 0 {
			switch {
			case trimmed == layoutTableHeader:
				if !inSection {
					return nil, fmt.Errorf("CLAUDE.md:%d: a %q table opens outside the %q section — every Layout table belongs under that heading, where its rows are measured: %q",
						at, layoutTableHeader, layoutHeading, trimmed)
				}
				if err := requireSeparator(lines, i, layoutTableHeader); err != nil {
					return nil, err
				}
				i++ // the separator belongs to this header and is not a data row
				inTable, layoutTbl, sawTable, tblHeader = true, true, true, layoutTableHeader
				prevProse = false
				continue

			case declaredNonLayoutHeader(trimmed) != "":
				if inSection {
					return nil, fmt.Errorf("CLAUDE.md:%d: a %q table inside the %q section — every table under that heading is a Layout table opening with %q, so its rows are measured as Layout rows: %q",
						at, trimmed, layoutHeading, layoutTableHeader, trimmed)
				}
				if err := requireSeparator(lines, i, trimmed); err != nil {
					return nil, err
				}
				i++
				inTable, layoutTbl, sawTable, tblHeader = true, false, true, trimmed
				prevProse = false
				continue

			case inTable && layoutTbl:
				m := layoutRowRe.FindStringSubmatch(trimmed)
				if m == nil {
					return nil, fmt.Errorf("CLAUDE.md:%d: line starts with %q but is neither a %q header, a %q separator, nor a well-formed data row (%q) — it would be skipped, and a skipped row escapes the cell-length and path-existence checks: %q",
						at, "|", layoutTableHeader, layoutTableSeparator, "| `path.go` | Description. |", trimmed)
				}
				paths := extractPaths(m[1])
				if len(paths) == 0 {
					return nil, fmt.Errorf("CLAUDE.md:%d: Layout row's first column has no backtick-quoted path: %q", at, m[1])
				}
				out.rows = append(out.rows, layoutRow{line: at, paths: paths, description: strings.TrimSpace(m[2])})
				prevProse = false
				continue

			case inTable:
				for col, cell := range splitRowCells(raw, pipes) {
					out.otherCells = append(out.otherCells, tableCell{line: at, header: tblHeader, column: col + 1, text: strings.TrimSpace(cell)})
				}
				prevProse = false
				continue

			default:
				return nil, fmt.Errorf("CLAUDE.md:%d: a line the Markdown table syntax reads as a table row, outside any declared table — every table opens with one of these headers (%s) followed by its %q separator, so a table spelled any other way, or placed anywhere else, holds cells no check measures: %q",
					at, declaredHeaders(), layoutTableSeparator, trimmed)
			}
		}

		if htmlBlockRe.MatchString(raw) {
			return nil, fmt.Errorf("CLAUDE.md:%d: an HTML block in CLAUDE.md — this guard reads Markdown structure only, so an HTML table or comment holds text that reads as documentation while escaping every row check: %q",
				at, trimmed)
		}

		if inSection && sawTable {
			return nil, fmt.Errorf("CLAUDE.md:%d: prose inside the %q section after a table has opened — only the section's own lead prose, ahead of the first table, may sit here; anything later reads as row text while escaping the cell-length and path-existence checks: %q",
				at, layoutHeading, trimmed)
		}

		prevProse = true // the section's own lead prose, or prose outside it
	}

	if fence != "" {
		return nil, fmt.Errorf("CLAUDE.md:%d: a code fence opened here is never closed, so every line beneath it is read as fenced content and measured by nothing", fenceAt)
	}
	if headings == 0 {
		return nil, fmt.Errorf("no %q heading in CLAUDE.md — the section moved or was renamed; fix parseCLAUDEMD in this test", layoutHeading)
	}
	if len(out.rows) == 0 {
		return nil, fmt.Errorf("parsed zero rows from CLAUDE.md's Layout table — the table moved or its format changed; fix parseCLAUDEMD in this test")
	}
	return &out, nil
}

// parseLayoutRows is parseCLAUDEMD's Layout-row half, the shape most fixtures
// assert on.
func parseLayoutRows(content string) ([]layoutRow, error) {
	parsed, err := parseCLAUDEMD(content)
	if err != nil {
		return nil, err
	}
	return parsed.rows, nil
}

func extractPaths(col1 string) []string {
	matches := backtickPathRe.FindAllStringSubmatch(col1, -1)
	paths := make([]string, 0, len(matches))
	for _, m := range matches {
		paths = append(paths, m[1])
	}
	return paths
}

// TestCLAUDEMDLayoutStaysCompact is the regression guard described above.
// Each failure names the row or file to fix, so its output doubles as the
// trim worklist.
func TestCLAUDEMDLayoutStaysCompact(t *testing.T) {
	data, err := os.ReadFile(claudeMDPath)
	require.NoErrorf(t, err, "could not read %s from the test's working directory (expected to be the repository root)", claudeMDPath)
	content := string(data)

	parsed, err := parseCLAUDEMD(content)
	require.NoError(t, err, "CLAUDE.md's Layout table did not parse")
	rows := parsed.rows

	t.Run("file size budget", func(t *testing.T) {
		require.LessOrEqualf(t, len(data), claudeMDMaxBytes,
			"%s is %d bytes, over the %d byte budget — trim a Layout table row back to a short pointer instead of appending prose to it", claudeMDPath, len(data), claudeMDMaxBytes)
	})

	t.Run("layout cell length budget", func(t *testing.T) {
		for _, row := range rows {
			t.Run(strings.Join(row.paths, " "), func(t *testing.T) {
				n := utf8.RuneCountInString(row.description)
				require.LessOrEqualf(t, n, layoutCellMaxChars,
					"CLAUDE.md:%d: Layout row for %v has a %d-character description cell, over the %d budget — trim it to a short pointer and move the detail into the file or doc it names", row.line, row.paths, n, layoutCellMaxChars)
			})
		}
	})

	t.Run("every table cell is within the cell budget", func(t *testing.T) {
		for _, cell := range parsed.otherCells {
			n := utf8.RuneCountInString(cell.text)
			require.LessOrEqualf(t, n, layoutCellMaxChars,
				"CLAUDE.md:%d: column %d of a %q row has a %d-character cell, over the %d budget — trim it to a short pointer", cell.line, cell.column, cell.header, n, layoutCellMaxChars)
		}
	})

	t.Run("every root go file has a layout row", func(t *testing.T) {
		covered := coveredPaths(rows, ".go")

		names, err := coverageTargets(".", isRootGoFile, rootGoFileCause)
		require.NoError(t, err)

		for _, name := range names {
			t.Run(name, func(t *testing.T) {
				_, ok := covered[name]
				require.Truef(t, ok, "%s has no row in CLAUDE.md's Layout table — add one describing its responsibility", name)
			})
		}
	})

	// The mirror of the check above, in the other direction CLAUDE.md points:
	// a design doc with no Layout row is one the file never names, so nothing
	// tells a reader it exists or what it owns.
	t.Run("every design doc has a layout row", func(t *testing.T) {
		covered := coveredPaths(rows, designDocSuffix)

		names, err := coverageTargets(designDocDir, isDesignDoc, designDocCause)
		require.NoError(t, err)

		for _, name := range names {
			path := filepath.ToSlash(filepath.Join(designDocDir, name))
			t.Run(path, func(t *testing.T) {
				_, ok := covered[path]
				require.Truef(t, ok, "%s has no row in CLAUDE.md's Layout table — add one describing what it owns", path)
			})
		}
	})

	t.Run("every layout path exists", func(t *testing.T) {
		for _, row := range rows {
			for _, p := range row.paths {
				t.Run(p, func(t *testing.T) {
					_, err := os.Stat(p)
					require.NoErrorf(t, err, "CLAUDE.md:%d: Layout row names %q, which does not exist on disk — update or remove the row", row.line, p)
				})
			}
		}
	})
}

// isRootGoFile and isDesignDoc are the two coverage filters, each spelled as a
// named predicate so the subtest and the emptiness guard below run over the
// same rule rather than two copies of it.
func isRootGoFile(name string) bool {
	return strings.HasSuffix(name, ".go") && !strings.HasSuffix(name, "_test.go")
}

func isDesignDoc(name string) bool {
	return strings.HasSuffix(name, designDocSuffix)
}

// rootGoFileCause and designDocCause name the likely reason each enumeration
// came back empty, so the refusal points at the edit that emptied it instead of
// only reporting a count.
const (
	rootGoFileCause = "the package's non-test .go files moved out of the repository root, or this test no longer runs from it"
	designDocCause  = "the design docs moved into a subdirectory of " + designDocDir +
		", or stopped matching the " + designDocSuffix + " suffix CLAUDE.md's Conventions declare"
)

// coverageTargets lists the entries of dir that pass keep, and REFUSES an empty
// result.
//
// Both callers assert one Layout row per entry, so an enumeration matching
// nothing runs zero per-entry assertions and reports PASS while covering
// nothing — the coverage claim silently lost, and every file added afterwards
// unguarded. Deleting every subject is the loud way to reach that state; the
// reachable way is an ordinary reorganisation, because the enumeration is a
// FLAT, non-recursive read with a literal suffix filter. Moving the design docs
// into docs/<subdir>/ and repointing their Layout rows, or renaming them to
// another convention, empties this set while every other subtest stays green.
// Hence the refusal, and hence a cause string rather than a bare count.
func coverageTargets(dir string, keep func(string) bool, cause string) ([]string, error) {
	entries, err := os.ReadDir(dir)
	if err != nil {
		return nil, fmt.Errorf("could not list %s: %w", dir, err)
	}
	var names []string
	for _, entry := range entries {
		if entry.IsDir() || !keep(entry.Name()) {
			continue
		}
		names = append(names, entry.Name())
	}
	if len(names) == 0 {
		return nil, fmt.Errorf("no entry of %s matched this coverage check's filter, so it would assert nothing and pass vacuously — %s", dir, cause)
	}
	return names, nil
}

// coveredPaths collects the paths the Layout rows name that carry the given
// suffix.
func coveredPaths(rows []layoutRow, suffix string) map[string]struct{} {
	covered := map[string]struct{}{}
	for _, row := range rows {
		for _, p := range row.paths {
			if strings.HasSuffix(p, suffix) {
				covered[p] = struct{}{}
			}
		}
	}
	return covered
}

// TestCoverageTargetsRefusesAnEmptyEnumeration pins the one thing the two
// coverage subtests cannot show about themselves: each one asserts a Layout row
// per enumerated entry, so an enumeration that matches nothing asserts nothing
// and reports PASS. Every fixture below is a directory an ordinary
// reorganisation could produce — no file deleted, the subjects merely moved or
// renamed — and each must be REFUSED, naming the cause.
//
// Every fixture is invented and lives in a temporary directory: none of these
// names appears in the repository.
func TestCoverageTargetsRefusesAnEmptyEnumeration(t *testing.T) {
	const inventedCause = "an invented cause this fixture names"

	write := func(t *testing.T, path string) {
		t.Helper()
		require.NoError(t, os.MkdirAll(filepath.Dir(path), 0o750))
		require.NoError(t, os.WriteFile(path, []byte("invented fixture content\n"), 0o600))
	}

	t.Run("design docs moved into a subdirectory", func(t *testing.T) {
		dir := t.TempDir()
		write(t, filepath.Join(dir, "design", "invented"+designDocSuffix))
		write(t, filepath.Join(dir, "invented-note.md"))

		names, err := coverageTargets(dir, isDesignDoc, inventedCause)
		require.Error(t, err, "an enumeration matching nothing must be refused, never left to assert nothing and pass")
		require.Nil(t, names)
		require.Contains(t, err.Error(), inventedCause, "the refusal must name the likely cause, not only the empty count")
	})

	t.Run("root go files moved out of the root", func(t *testing.T) {
		dir := t.TempDir()
		write(t, filepath.Join(dir, "internal", "invented.go"))
		write(t, filepath.Join(dir, "invented_test.go"))

		names, err := coverageTargets(dir, isRootGoFile, inventedCause)
		require.Error(t, err, "a root holding only test files must be refused, since the check would cover nothing")
		require.Nil(t, names)
		require.Contains(t, err.Error(), inventedCause)
	})

	t.Run("a missing directory is refused loudly", func(t *testing.T) {
		names, err := coverageTargets(filepath.Join(t.TempDir(), "no_such_invented_dir"), isDesignDoc, inventedCause)
		require.Error(t, err)
		require.Nil(t, names)
	})

	t.Run("accepts the matching entries and drops the rest", func(t *testing.T) {
		dir := t.TempDir()
		write(t, filepath.Join(dir, "invented-a"+designDocSuffix))
		write(t, filepath.Join(dir, "invented-b"+designDocSuffix))
		write(t, filepath.Join(dir, "invented-note.md"))
		write(t, filepath.Join(dir, "design", "invented-c"+designDocSuffix))

		names, err := coverageTargets(dir, isDesignDoc, inventedCause)
		require.NoError(t, err)
		require.Equal(t, []string{"invented-a" + designDocSuffix, "invented-b" + designDocSuffix}, names,
			"the filter keeps matching files in ReadDir order and drops directories and non-matching names")
	})

	t.Run("the real enumerations are not empty", func(t *testing.T) {
		goFiles, err := coverageTargets(".", isRootGoFile, rootGoFileCause)
		require.NoError(t, err)
		require.NotEmpty(t, goFiles)

		docs, err := coverageTargets(designDocDir, isDesignDoc, designDocCause)
		require.NoError(t, err)
		require.NotEmpty(t, docs)
	})
}

// TestParseLayoutRowsRejectsMalformedRows pins the one thing the budget test
// above cannot show about itself: a pipe-prefixed Layout line that fails to
// parse is REFUSED, not skipped. A skipped line is invisible to the
// cell-length and path-existence checks, so a row missing its closing "|"
// could carry an arbitrarily long description and name a path that does not
// exist while the whole test still reported PASS.
//
// Every fixture below is invented — none of these lines appears in the real
// CLAUDE.md, and none should ever be copied into it.
func TestParseLayoutRowsRejectsMalformedRows(t *testing.T) {
	// section wraps table body lines in the minimum structure the parser
	// needs: the "## Layout" heading it starts at, a sub-heading and table
	// scaffolding, and a following "##" heading it stops at.
	section := func(body ...string) string {
		lines := []string{layoutHeading, "", inventedSubHeading, "", layoutTableHeader, layoutTableSeparator}
		lines = append(lines, body...)
		return strings.Join(append(lines, "", "## Conventions", ""), "\n")
	}

	const wellFormed = "| `doc.go` | An invented short pointer row. |"

	// oversized is one rune past layoutCellMaxChars, so a fixture carrying
	// it fails the cell budget the moment it is parsed as a row.
	oversized := strings.Repeat("X", layoutCellMaxChars+1)

	t.Run("accepts the well-formed shape", func(t *testing.T) {
		rows, err := parseLayoutRows(section(wellFormed))
		require.NoError(t, err)
		require.Len(t, rows, 1, "the header and separator must still be skipped, and the data row kept")
		require.Equal(t, []string{"doc.go"}, rows[0].paths)
		require.Equal(t, "An invented short pointer row.", rows[0].description)
	})

	for name, malformed := range map[string]string{
		// The audited defect: a docs row whose closing "|" is missing, so
		// layoutRowRe's "\|$" anchor cannot match and the line was skipped.
		"row missing its closing pipe": "| `docs/invented-design.md` | An invented pointer row with no closing delimiter",
		// The same shape carrying a cell far past the budget — proof that a
		// skipped row really did escape the cap.
		"oversized cell on a row missing its closing pipe": "| `docs/invented-design.md` | " + oversized,
		"first column is not a backtick-quoted path":       "| docs/invented-design.md | An invented pointer row. |",
		"first column holds three backtick tokens":         "| `a.go` / `b.go` / `c.go` | An invented pointer row. |",
		"pipe-prefixed prose with no second column":        "| An invented stray line |",
	} {
		t.Run(name, func(t *testing.T) {
			rows, err := parseLayoutRows(section(wellFormed, malformed))
			require.Error(t, err, "a malformed Layout line must be refused, never skipped")
			require.Nil(t, rows)
			require.Contains(t, err.Error(), malformed, "the failure must quote the offending line so an author can find it")
		})
	}

	t.Run("an empty table is still refused", func(t *testing.T) {
		_, err := parseLayoutRows(section())
		require.Error(t, err)
		require.Contains(t, err.Error(), "parsed zero rows")
	})
}

// TestParseLayoutRowsRefusesTextItWouldNotReach pins the parser's REACH, which
// the budget test cannot show about itself either: a guard that binds to one
// heading and stops at the next measures only what it happens to visit, so
// text placed anywhere else carries an unbounded cell and names whatever path
// it likes while the whole test still reports PASS. Each fixture below is a
// placement that would go unvisited by a parser that stopped early, and each
// must be REFUSED rather than left unread.
//
// Every fixture is invented — none of these lines appears in the real
// CLAUDE.md, and none should ever be copied into it. Each oversized fixture
// row is doubly out of contract: its cell runs past layoutCellMaxChars AND it
// names a file that does not exist, so a parse that reached it could not
// possibly pass.
func TestParseLayoutRowsRefusesTextItWouldNotReach(t *testing.T) {
	const wellFormed = "| `doc.go` | An invented short pointer row. |"

	oversized := "| `no_such_invented_file.go` | " + strings.Repeat("X", layoutCellMaxChars+1) + " |"

	// layoutSection renders one whole "## Layout" section — its heading, a
	// sub-heading, table scaffolding, and the given rows.
	layoutSection := func(rows ...string) []string {
		lines := []string{layoutHeading, "", inventedSubHeading, "", layoutTableHeader, layoutTableSeparator}
		return append(append(lines, rows...), "")
	}
	join := func(blocks ...[]string) string {
		var lines []string
		for _, block := range blocks {
			lines = append(lines, block...)
		}
		return strings.Join(lines, "\n")
	}
	// other is any following section, so the Layout section really does end.
	other := []string{"## Conventions", "", "An invented sentence.", ""}

	for name, tc := range map[string]struct {
		content string
		want    string // substring the refusal must carry
	}{
		// The audited defect: an annex section appended past the real one,
		// which a parser bound to the FIRST heading never visits.
		"a second Layout heading after the section": {
			content: join(layoutSection(wellFormed), other, layoutSection(oversized)),
			want:    "a second",
		},
		// The same defect the other way round, and worse: a decoy placed
		// FIRST captures the parse, leaving the real table unread.
		"a decoy Layout heading before the section": {
			content: join(layoutSection(oversized), other, layoutSection(wellFormed)),
			want:    "a second",
		},
		// The duplicate fenced off, so it feeds the parser a second heading
		// while rendering as a code block rather than as a section.
		"a Layout heading hidden in a code fence": {
			content: join(layoutSection(wellFormed), other, []string{"```markdown"}, layoutSection(oversized), []string{"```", ""}),
			want:    "a second",
		},
		"a Layout table outside the Layout section": {
			content: join(layoutSection(wellFormed), []string{"## Conventions", "", layoutTableHeader, layoutTableSeparator, oversized, ""}),
			want:    "outside the",
		},
		"prose beside the rows": {
			content: join(layoutSection(wellFormed, "", "An invented aside, as long as its author likes."), other),
			want:    "prose inside",
		},
		// An HTML comment is refused as an HTML block now that the scanner
		// classifies one, a rule that reaches the whole file rather than the
		// section alone. The refusal it carried before, "prose inside", is
		// pinned by the fixture above.
		"an HTML comment beside the rows": {
			content: join(layoutSection(wellFormed, "", "<!-- an invented aside -->"), other),
			want:    "an HTML block",
		},
		"a row stranded outside any table": {
			content: join(layoutSection(wellFormed, "", oversized), other),
			want:    wantStrayTable,
		},
		"a table header with no separator beneath it": {
			content: join([]string{layoutHeading, "", layoutTableHeader, wellFormed, ""}, other),
			want:    "separator",
		},
		"no Layout heading at all": {
			content: join(other),
			want:    `no "## Layout" heading`,
		},
	} {
		t.Run(name, func(t *testing.T) {
			rows, err := parseLayoutRows(tc.content)
			require.Error(t, err, "text the parser would not otherwise reach must be refused, never left unread")
			require.Nil(t, rows)
			require.Contains(t, err.Error(), tc.want)
		})
	}

	t.Run("the section's own lead prose is accepted", func(t *testing.T) {
		content := join([]string{
			layoutHeading, "",
			"An invented lead sentence, ahead of the first table.", "",
			inventedSubHeading, "",
			layoutTableHeader, layoutTableSeparator, wellFormed, "",
		}, other)
		rows, err := parseLayoutRows(content)
		require.NoError(t, err, "prose ahead of the first table is the one prose the section carries")
		require.Len(t, rows, 1)
	})
}

// TestParseCLAUDEMDRefusesEverySpellingOfTheAnchors is the root-cause
// regression: the guard used to recognise the Layout heading and its tables by
// exact-string and single-spelling tests on individually trimmed lines, with
// `continue` as the default for anything else, so every ALTERNATIVE Markdown
// spelling of a heading or a table walked past it. Each fixture below is one
// such spelling, and each must be refused by the total classifier rather than
// skipped.
//
// Every fixture is invented — none of these lines appears in the real
// CLAUDE.md, and none should ever be copied into it. Each carries the same
// doubly-out-of-contract row: a cell one rune past layoutCellMaxChars naming a
// file that does not exist, so any parse that reached it could not pass.
func TestParseCLAUDEMDRefusesEverySpellingOfTheAnchors(t *testing.T) {
	oversized := "| `no_such_invented_file.go` | " + strings.Repeat("X", layoutCellMaxChars+1) + " |"

	// realSection is a complete, well-formed Layout section, so every fixture
	// below fails on its own escape rather than on a missing heading.
	realSection := []string{
		layoutHeading, "",
		inventedSubHeading, "",
		layoutTableHeader, layoutTableSeparator,
		"| `doc.go` | An invented short pointer row. |", "",
	}
	other := []string{"## Conventions", "", "An invented sentence.", ""}
	join := func(blocks ...[]string) string {
		var lines []string
		for _, block := range blocks {
			lines = append(lines, block...)
		}
		return strings.Join(lines, "\n")
	}

	// invented non-canonical table shapes, each holding the same oversized row.
	noPipeTable := []string{"Path | Responsibility", "---|---", "`no_such_invented_file.go` | " + strings.Repeat("X", layoutCellMaxChars+1), ""}

	for name, tc := range map[string]struct {
		content string
		want    string
	}{
		// Axis A — every other way to delimit the section.
		"setext Layout heading underlined with dashes": {
			content: join(realSection, other, []string{"Layout", "------", ""}, []string{layoutTableHeader, layoutTableSeparator, oversized, ""}),
			want:    "setext heading",
		},
		"setext Layout heading underlined with one dash": {
			content: join(realSection, other, []string{"Layout", "-", ""}),
			want:    "setext heading",
		},
		"setext Layout heading underlined with equals": {
			content: join(realSection, other, []string{"Layout", "======", ""}),
			want:    "setext heading",
		},
		"ATX heading with a closing hash sequence": {
			content: join(realSection, other, []string{"## Layout ##", "", layoutTableHeader, layoutTableSeparator, oversized, ""}),
			want:    wantMisspelledHeading,
		},
		"ATX heading with two spaces after the hashes": {
			content: join(realSection, other, []string{"##  Layout", ""}),
			want:    wantMisspelledHeading,
		},
		"ATX heading with a tab after the hashes": {
			content: join(realSection, other, []string{"##\tLayout", ""}),
			want:    wantMisspelledHeading,
		},
		"ATX heading in lower case": {
			content: join(realSection, other, []string{"## layout", ""}),
			want:    wantMisspelledHeading,
		},
		"ATX heading in upper case": {
			content: join(realSection, other, []string{"## LAYOUT", ""}),
			want:    wantMisspelledHeading,
		},
		"ATX heading separated by a no-break space": {
			content: join(realSection, other, []string{"## Layout", ""}),
			want:    wantMisspelledHeading,
		},
		"no heading at all, a table appended under another section": {
			content: join(realSection, other, []string{"| File | Rule |", "|---|---|", oversized, ""}),
			want:    wantStrayTable,
		},

		// Axis B — every other way to delimit the table.
		"another two-column header outside the section": {
			content: join(realSection, other, []string{"| Module | Notes |", "|---|---|", oversized, ""}),
			want:    wantStrayTable,
		},
		"a row indented out of its own table": {
			content: join([]string{
				layoutHeading, "", inventedSubHeading, "",
				layoutTableHeader, layoutTableSeparator,
				"| `doc.go` | An invented short pointer row. |",
				"    " + oversized, "",
			}, other),
			want: "indented four spaces or more",
		},
		"the Layout header in another case, outside the section": {
			content: join(realSection, other, []string{"| path | responsibility |", "|---|---|", oversized, ""}),
			want:    wantStrayTable,
		},
		"a three-column header outside the section": {
			content: join(realSection, other, []string{"| Path | Responsibility | Notes |", "|---|---|---|", oversized, ""}),
			want:    wantStrayTable,
		},
		"a GFM table with no leading pipes, outside the section": {
			content: join(realSection, other, noPipeTable),
			want:    wantStrayTable,
		},
		"an HTML table outside the section": {
			content: join(realSection, other, []string{"<table><tr><td>" + strings.Repeat("X", layoutCellMaxChars+1) + "</td></tr></table>", ""}),
			want:    "an HTML block",
		},

		// Axis C — every other place the text can sit.
		"a GFM table with no leading pipes in the section's lead": {
			content: join([]string{layoutHeading, ""}, noPipeTable, []string{inventedSubHeading, "", layoutTableHeader, layoutTableSeparator, "| `doc.go` | An invented short pointer row. |", ""}, other),
			want:    wantStrayTable,
		},
		"an HTML table in the section's lead": {
			content: join([]string{layoutHeading, "", "<table><tr><td>an invented cell</td></tr></table>", "", inventedSubHeading, "", layoutTableHeader, layoutTableSeparator, "| `doc.go` | An invented short pointer row. |", ""}, other),
			want:    "an HTML block",
		},
		"a non-canonical table before the Layout section": {
			content: join([]string{"# CLAUDE.md", "", "| File | Rule |", "|---|---|", oversized, ""}, realSection, other),
			want:    wantStrayTable,
		},
		"a declared non-Layout table moved inside the Layout section": {
			content: join([]string{layoutHeading, "", nonLayoutTableHeaders[0].header, layoutTableSeparator, "| An invented cell | Another. |", ""}, other),
			want:    "inside the",
		},

		// The two anchors, fenced.
		"the Layout table header hidden in a code fence": {
			content: join(realSection, other, []string{"```markdown", layoutTableHeader, "```", ""}),
			want:    "inside a code fence",
		},
		"a code fence left open at EOF": {
			content: join(realSection, other, []string{"```markdown", oversized}),
			want:    "never closed",
		},
	} {
		t.Run(name, func(t *testing.T) {
			rows, err := parseLayoutRows(tc.content)
			require.Error(t, err, "every alternative spelling of a heading or a table must be refused, never skipped")
			require.Nil(t, rows)
			require.Contains(t, err.Error(), tc.want)
		})
	}
}

// TestParseCLAUDEMDAcceptsTheShapesTheFormatMakesSafe pins the other half of
// carrying fence state: the shapes an earlier, trim-everything scanner refused
// as false positives, which knowing a fence is a fence and reading the
// format's own three-space indent rule make safe to accept.
//
// Every fixture is invented — none of these lines appears in the real
// CLAUDE.md.
func TestParseCLAUDEMDAcceptsTheShapesTheFormatMakesSafe(t *testing.T) {
	const wellFormed = "| `doc.go` | An invented short pointer row. |"
	other := []string{"## Conventions", "", "An invented sentence.", ""}
	join := func(blocks ...[]string) string {
		var lines []string
		for _, block := range blocks {
			lines = append(lines, block...)
		}
		return strings.Join(lines, "\n")
	}
	realSection := []string{
		layoutHeading, "",
		inventedSubHeading, "",
		layoutTableHeader, layoutTableSeparator, wellFormed, "",
	}

	for name, content := range map[string]string{
		// An over-indented copy renders as code and, under the format's
		// three-space rule, is not the canonical spelling at all.
		"an indented Layout heading is not a heading": join(realSection, other, []string{"    ## Layout", ""}),
		// Inside the section, after a table: a deeper sub-heading and a
		// fenced code example are structure and code, not prose beside rows.
		"a deeper sub-heading after a table": join([]string{
			layoutHeading, "", inventedSubHeading, "",
			layoutTableHeader, layoutTableSeparator, wellFormed, "",
			"#### Invented note", "",
			layoutTableHeader, layoutTableSeparator, "| `errors.go` | Another invented row. |", "",
		}, other),
		"a fenced code example after a table": join([]string{
			layoutHeading, "", inventedSubHeading, "",
			layoutTableHeader, layoutTableSeparator, wellFormed, "",
			"```go", "func invented() {}", "```", "",
		}, other),
		// A thematic break is not a setext underline: nothing precedes it.
		"a thematic break outside the section": join(realSection, other, []string{"---", ""}),
		// The HTML-block rule is LINE-LEVEL. htmlBlockRe only ever sees a
		// line-start delimiter, so a `<` in the middle of a prose line is
		// text this scanner never classifies. CLAUDE.md's Conventions
		// section states the rule at exactly that width, and this case with
		// the standalone-comment refusal above are what hold it there.
		"an HTML comment inside a prose line": join(realSection, []string{
			"## Conventions", "",
			"An invented sentence with <!-- an invented aside --> inside it.", "",
		}),
		// The declared non-Layout table, where it belongs.
		"a declared non-Layout table outside the section": join([]string{
			nonLayoutTableHeaders[0].section, "",
			nonLayoutTableHeaders[0].header, layoutTableSeparator,
			"| An invented cell | Another. |", "",
		}, realSection, other),
	} {
		t.Run(name, func(t *testing.T) {
			rows, err := parseLayoutRows(content)
			require.NoError(t, err, "the format makes this shape safe to accept")
			require.NotEmpty(t, rows)
		})
	}

	t.Run("a declared non-Layout table's cells are still measured", func(t *testing.T) {
		oversizedCell := strings.Repeat("X", layoutCellMaxChars+1)
		parsed, err := parseCLAUDEMD(join([]string{
			nonLayoutTableHeaders[0].section, "",
			nonLayoutTableHeaders[0].header, layoutTableSeparator,
			"| An invented cell | " + oversizedCell + " |", "",
		}, realSection, other))
		require.NoError(t, err)
		require.Len(t, parsed.otherCells, 2, "declaring a table bounds it rather than exempting it")
		require.Equal(t, oversizedCell, parsed.otherCells[1].text,
			"the cell budget subtest must see this cell, since a declared table's cells are measured like a Layout row's")
	})
}
