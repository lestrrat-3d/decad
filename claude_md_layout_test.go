package decad_test

import (
	"fmt"
	"os"
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
// The guard is only as strong as the parser's reach, so parseLayoutRows reads
// the whole file and refuses every structure that would put text beside the
// rows without being measured as one — a second "## Layout" heading, a Layout
// table outside that section, prose or a comment among the tables, a row
// belonging to no table. Its own doc comment states those rules; the byte
// budget above is what bounds everything else the file may hold.
const (
	claudeMDPath = "CLAUDE.md"
	// claudeMDMaxBytes is the whole-file budget. Raising it is a policy
	// change, not a bug fix — trim a Layout row instead.
	claudeMDMaxBytes = 32768
	// layoutCellMaxChars caps a Layout row's description (second) column.
	// A row that needs more detail belongs in the file/doc it names, with
	// this cell trimmed back to a short pointer.
	layoutCellMaxChars = 300
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

// layoutHeaderRe and layoutSeparatorRe match the only two non-data lines a
// Layout sub-table may open with: its "| Path | Responsibility |" header and
// the "|---|---|" alignment row beneath it (optional padding and alignment
// colons allowed). They exist so that parseLayoutRows can name exactly what it
// is allowed to skip: every OTHER pipe-prefixed line in the section is a data
// row, and one that fails layoutRowRe is malformed rather than scaffolding.
var (
	layoutHeaderRe    = regexp.MustCompile(`^\|\s*Path\s*\|\s*Responsibility\s*\|$`)
	layoutSeparatorRe = regexp.MustCompile(`^\|(?:\s*:?-+:?\s*\|)+$`)
)

// layoutHeading is the one "##" heading CLAUDE.md's Layout tables live under.
// The file may carry exactly one of it: parseLayoutRows binds its whole parse
// to that heading, so a second copy anywhere in the file — before the real one
// as a decoy, after it as an annex, fenced off so it renders invisibly — would
// silently decide which rows are measured and which are never seen.
const layoutHeading = "## Layout"

// layoutTableHeader and layoutTableSeparator are the two scaffolding lines a
// Layout table opens with, spelled once so the refusals, the fixtures and
// layoutHeaderRe/layoutSeparatorRe above cannot drift apart.
const (
	layoutTableHeader    = "| Path | Responsibility |"
	layoutTableSeparator = "|---|---|"
)

// layoutRow is one parsed data row of CLAUDE.md's "## Layout" table.
type layoutRow struct {
	line        int      // 1-based source line, for failure messages
	paths       []string // one or two paths named in column 1
	description string   // raw column 2 text
}

// parseLayoutRows extracts every data row of every "## Layout" markdown table.
//
// It reads the WHOLE file rather than stopping at the section's end, because
// the checks its rows feed are only as good as the parser's reach: text the
// parser never visits carries an unbounded description cell and names whatever
// path it likes with nothing measuring either. Four structural rules keep that
// reach total, each refusing rather than skipping:
//
//  1. layoutHeading appears exactly once. A second copy — earlier, later, or
//     inside a code fence — would otherwise capture the parse and leave the
//     rows it did not capture unmeasured.
//  2. Every "| Path | Responsibility |" table opens inside that section, and
//     its separator row follows immediately. A Layout table anywhere else is
//     refused instead of ignored.
//  3. Inside the section, the only prose allowed is the section's own lead,
//     ahead of the first table. Once a table has opened, a line that is not a
//     row, a blank, or a "###" sub-heading — loose prose, an HTML comment, a
//     fence marker — is refused, since prose beside the rows reads as row text
//     while escaping every per-row check.
//  4. Every pipe-prefixed line belongs to an open table and parses as a row.
//     A row missing its closing "|", one whose first column is not a
//     backtick-quoted path, and one stranded outside any table are all
//     refusals.
func parseLayoutRows(content string) ([]layoutRow, error) {
	var rows []layoutRow
	var (
		headings  int  // "## Layout" headings seen so far
		inSection bool // inside the Layout section
		inTable   bool // inside an open Layout table's row block
		sawTable  bool // a Layout table has opened in this section
	)

	lines := strings.Split(content, "\n")
	for i := 0; i < len(lines); i++ {
		trimmed := strings.TrimSpace(lines[i])

		// The Layout section groups its rows into several tables under "###"
		// sub-headings, so it ends at the next "##" heading rather than at the
		// first blank line after a table.
		if strings.HasPrefix(trimmed, "## ") {
			inSection = trimmed == layoutHeading
			inTable, sawTable = false, false
			if inSection {
				headings++
				if headings > 1 {
					return nil, fmt.Errorf("CLAUDE.md:%d: a second %q heading — the file may carry exactly one, since a duplicate captures the parse and leaves every row of the other section unmeasured: %q",
						i+1, layoutHeading, trimmed)
				}
			}
			continue
		}

		if layoutHeaderRe.MatchString(trimmed) {
			if !inSection {
				return nil, fmt.Errorf("CLAUDE.md:%d: a %q table opens outside the %q section — every Layout table belongs under that heading, where its rows are measured: %q",
					i+1, layoutTableHeader, layoutHeading, trimmed)
			}
			if i+1 >= len(lines) || !layoutSeparatorRe.MatchString(strings.TrimSpace(lines[i+1])) {
				return nil, fmt.Errorf("CLAUDE.md:%d: %q header is not followed by its %q separator, so the lines beneath it are not a table at all",
					i+1, layoutTableHeader, layoutTableSeparator)
			}
			i++ // the separator belongs to this header and is not a data row
			inTable, sawTable = true, true
			continue
		}

		if !inSection {
			continue
		}

		switch {
		case trimmed == "":
			inTable = false // a blank line closes the table it followed
			continue
		case strings.HasPrefix(trimmed, "### "):
			inTable = false
			continue
		}

		if !strings.HasPrefix(trimmed, "|") {
			if sawTable {
				return nil, fmt.Errorf("CLAUDE.md:%d: prose inside the %q section after a table has opened — only the section's own lead prose, ahead of the first table, may sit here; anything later reads as row text while escaping the cell-length and path-existence checks: %q",
					i+1, layoutHeading, trimmed)
			}
			continue // the section's own lead prose
		}

		if !inTable {
			return nil, fmt.Errorf("CLAUDE.md:%d: table row outside any %q table — a stranded row is measured by nothing: %q",
				i+1, layoutTableHeader, trimmed)
		}

		m := layoutRowRe.FindStringSubmatch(trimmed)
		if m == nil {
			return nil, fmt.Errorf("CLAUDE.md:%d: line starts with %q but is neither a %q header, a %q separator, nor a well-formed data row (%q) — it would be skipped, and a skipped row escapes the cell-length and path-existence checks: %q",
				i+1, "|", layoutTableHeader, layoutTableSeparator, "| `path.go` | Description. |", trimmed)
		}

		paths := extractPaths(m[1])
		if len(paths) == 0 {
			return nil, fmt.Errorf("CLAUDE.md:%d: Layout row's first column has no backtick-quoted path: %q", i+1, m[1])
		}

		rows = append(rows, layoutRow{line: i + 1, paths: paths, description: strings.TrimSpace(m[2])})
	}

	if headings == 0 {
		return nil, fmt.Errorf("no %q heading in CLAUDE.md — the section moved or was renamed; fix parseLayoutRows in this test", layoutHeading)
	}
	if len(rows) == 0 {
		return nil, fmt.Errorf("parsed zero rows from CLAUDE.md's Layout table — the table moved or its format changed; fix parseLayoutRows in this test")
	}
	return rows, nil
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

	rows, err := parseLayoutRows(content)
	require.NoError(t, err, "CLAUDE.md's Layout table did not parse")

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

	t.Run("every root go file has a layout row", func(t *testing.T) {
		covered := map[string]struct{}{}
		for _, row := range rows {
			for _, p := range row.paths {
				if strings.HasSuffix(p, ".go") {
					covered[p] = struct{}{}
				}
			}
		}

		entries, err := os.ReadDir(".")
		require.NoError(t, err, "could not list the repository root")

		for _, entry := range entries {
			if entry.IsDir() {
				continue
			}
			name := entry.Name()
			if !strings.HasSuffix(name, ".go") || strings.HasSuffix(name, "_test.go") {
				continue
			}
			t.Run(name, func(t *testing.T) {
				_, ok := covered[name]
				require.Truef(t, ok, "%s has no row in CLAUDE.md's Layout table — add one describing its responsibility", name)
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
		lines := []string{layoutHeading, "", "### Invented sub-table", "", layoutTableHeader, layoutTableSeparator}
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
		lines := []string{layoutHeading, "", "### Invented sub-table", "", layoutTableHeader, layoutTableSeparator}
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
		"an HTML comment beside the rows": {
			content: join(layoutSection(wellFormed, "", "<!-- an invented aside -->"), other),
			want:    "prose inside",
		},
		"a row stranded outside any table": {
			content: join(layoutSection(wellFormed, "", oversized), other),
			want:    "outside any",
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
			"### Invented sub-table", "",
			layoutTableHeader, layoutTableSeparator, wellFormed, "",
		}, other)
		rows, err := parseLayoutRows(content)
		require.NoError(t, err, "prose ahead of the first table is the one prose the section carries")
		require.Len(t, rows, 1)
	})
}
