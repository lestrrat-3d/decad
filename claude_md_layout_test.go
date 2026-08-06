package decad_test

import (
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
// this test fails the moment the file grows past its budget or a row's cell
// grows past its own. A mechanical comparison that can FAIL beats a comment
// asking authors to "keep it short".
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

// layoutRow is one parsed data row of CLAUDE.md's "## Layout" table.
type layoutRow struct {
	line        int      // 1-based source line, for failure messages
	paths       []string // one or two paths named in column 1
	description string   // raw column 2 text
}

// parseLayoutRows extracts every data row of the "## Layout" markdown
// table (the header and separator rows don't match layoutRowRe, since
// neither "Path"/"Responsibility" nor "---" is a backtick-quoted path, so
// they're skipped rather than misread as data).
func parseLayoutRows(t *testing.T, content string) []layoutRow {
	t.Helper()

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

		m := layoutRowRe.FindStringSubmatch(trimmed)
		if m == nil {
			continue // the "| Path | Responsibility |" header or "|---|---|" separator
		}

		paths := extractPaths(m[1])
		require.NotEmptyf(t, paths, "CLAUDE.md:%d: Layout row's first column has no backtick-quoted path: %q", i+1, m[1])

		rows = append(rows, layoutRow{line: i + 1, paths: paths, description: strings.TrimSpace(m[2])})
	}

	require.NotEmpty(t, rows, "parsed zero rows from CLAUDE.md's Layout table — the table moved or its format changed; fix parseLayoutRows in this test")
	return rows
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
// It is expected to fail today: CLAUDE.md is well over budget and many
// Layout rows carry paragraphs of prose instead of a pointer. Each failure
// names the row/file to fix, so this test doubles as the trim campaign's
// worklist.
func TestCLAUDEMDLayoutStaysCompact(t *testing.T) {
	data, err := os.ReadFile(claudeMDPath)
	require.NoErrorf(t, err, "could not read %s from the test's working directory (expected to be the repository root)", claudeMDPath)
	content := string(data)

	rows := parseLayoutRows(t, content)

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
