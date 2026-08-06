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

// layoutRow is one parsed data row of CLAUDE.md's "## Layout" table.
type layoutRow struct {
	line        int      // 1-based source line, for failure messages
	paths       []string // one or two paths named in column 1
	description string   // raw column 2 text
}

// parseLayoutRows extracts every data row of the "## Layout" markdown table.
//
// The only pipe-prefixed lines it skips are the two it can NAME: each
// sub-table's "| Path | Responsibility |" header and its "|---|---|"
// separator. Anything else that starts with "|" and fails layoutRowRe is
// reported as a malformed row rather than skipped, because a skipped line is
// invisible to every check the caller then runs — a row missing its closing
// "|", or whose first column is not a backtick-quoted path, would otherwise
// carry an unbounded description cell and name a nonexistent path with
// nothing measuring either.
func parseLayoutRows(content string) ([]layoutRow, error) {
	var rows []layoutRow
	inSection := false

	for i, line := range strings.Split(content, "\n") {
		trimmed := strings.TrimSpace(line)

		if !inSection {
			if trimmed == "## Layout" {
				inSection = true
			}
			continue
		}

		// The Layout section groups its rows into several tables under "###"
		// sub-headings, so the section ends at the next "##" heading rather
		// than at the first blank line after a table.
		if strings.HasPrefix(trimmed, "## ") {
			break
		}

		if !strings.HasPrefix(trimmed, "|") {
			continue // a sub-heading, a blank line, or the section's own lead prose
		}

		if layoutHeaderRe.MatchString(trimmed) || layoutSeparatorRe.MatchString(trimmed) {
			continue // a sub-table's own header or separator row
		}

		m := layoutRowRe.FindStringSubmatch(trimmed)
		if m == nil {
			return nil, fmt.Errorf("CLAUDE.md:%d: line starts with %q but is neither a %q header, a %q separator, nor a well-formed data row (%q) — it would be skipped, and a skipped row escapes the cell-length and path-existence checks: %q",
				i+1, "|", "| Path | Responsibility |", "|---|---|", "| `path.go` | Description. |", trimmed)
		}

		paths := extractPaths(m[1])
		if len(paths) == 0 {
			return nil, fmt.Errorf("CLAUDE.md:%d: Layout row's first column has no backtick-quoted path: %q", i+1, m[1])
		}

		rows = append(rows, layoutRow{line: i + 1, paths: paths, description: strings.TrimSpace(m[2])})
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
		lines := []string{"## Layout", "", "### Invented sub-table", "", "| Path | Responsibility |", "|---|---|"}
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
