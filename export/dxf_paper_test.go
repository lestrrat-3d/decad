package export_test

import (
	"bytes"
	"strconv"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/dxf-export-design.md §6's tests for DXFSheet, on the
// 20 x 30 x 10 mm block of dxf_sheet_test.go.

// dxfText is one TEXT entity read back from a file.
type dxfText struct {
	layer, value string
	at           [2]float64
	height, turn float64
}

// readDXFTexts returns the TEXT entities of an ASCII DXF file.
func readDXFTexts(t *testing.T, file string) []dxfText {
	t.Helper()
	raw := strings.Split(strings.TrimRight(file, "\n"), "\n")
	var out []dxfText
	var cur *dxfText
	number := func(v string) float64 {
		f, err := strconv.ParseFloat(strings.TrimSpace(v), 64)
		require.NoError(t, err)
		return f
	}
	for i := 0; i+1 < len(raw); i += 2 {
		code, value := strings.TrimSpace(raw[i]), strings.TrimSpace(raw[i+1])
		switch {
		case code == "0":
			if cur != nil {
				out = append(out, *cur)
				cur = nil
			}
			if value == "TEXT" {
				cur = &dxfText{}
			}
		case cur == nil:
		case code == "8":
			cur.layer = value
		case code == "10":
			cur.at[0] = number(value)
		case code == "20":
			cur.at[1] = number(value)
		case code == "40":
			cur.height = number(value)
		case code == "1":
			cur.value = value
		case code == "50":
			cur.turn = number(value)
		}
	}
	if cur != nil {
		out = append(out, *cur)
	}
	return out
}

func a4Sheet(dimensions bool) export.DXFPaper {
	return export.DXFPaper{
		Views:  []export.DrawingView{topView, frontView},
		Width:  units.Millimeters(297),
		Height: units.Millimeters(210),
		Scale:  units.Scalar(0.5),
		Margin: units.Millimeters(10),
		Gap:    units.Millimeters(15),
		Title:  "BLOCK", Dimensions: dimensions,
	}
}

func TestDXFSheetScalesAndPlacesTheViewsInsideTheBorder(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	var out bytes.Buffer
	require.NoError(t, export.DXFSheet(t.Context(), &out, body, units.Millimeters(0.1), a4Sheet(true)))
	lines, sections := readDXFLines(t, out.String())
	require.Equal(t, []string{"HEADER", "TABLES", "ENTITIES"}, sections)
	byLayer := map[string][]dxfLine{}
	for _, l := range lines {
		byLayer[l.layer] = append(byLayer[l.layer], l)
	}
	// The border is the paper inset by the margin, with the title block's top line and divider.
	minX, maxX, minY, maxY := sheetBounds(byLayer[export.DXFBorderLayer])
	require.InDelta(t, 10, minX, 1e-9)
	require.InDelta(t, 287, maxX, 1e-9)
	require.InDelta(t, 10, minY, 1e-9)
	require.InDelta(t, 200, maxY, 1e-9)
	require.Len(t, byLayer[export.DXFBorderLayer], 6)
	// Row 35 wide plus 12 for the dimensions, centred in 277: the top view starts at 137.
	// The area above the title block is 176 deep, the row 15 and the dimensions 12: y = 110.5.
	var top, front []dxfLine
	for _, l := range byLayer[export.DXFVisibleLayer] {
		if l.a[0] < 150 && l.b[0] < 150 {
			top = append(top, l)
		} else {
			front = append(front, l)
		}
	}
	minX, maxX, minY, maxY = sheetBounds(top)
	require.InDelta(t, 137, minX, 1e-9)
	require.InDelta(t, 147, maxX, 1e-9, "20 mm at 1:2")
	require.InDelta(t, 110.5, minY, 1e-9)
	require.InDelta(t, 125.5, maxY, 1e-9, "30 mm at 1:2")
	minX, maxX, minY, maxY = sheetBounds(front)
	require.InDelta(t, 162, minX, 1e-9, "the top view's 10 mm and the 15 mm gap on")
	require.InDelta(t, 172, maxX, 1e-9)
	require.InDelta(t, 110.5, minY, 1e-9)
	require.InDelta(t, 115.5, maxY, 1e-9, "10 mm at 1:2")
	texts := readDXFTexts(t, out.String())
	var values []string
	for _, tx := range texts {
		values = append(values, tx.value)
	}
	// The dimensions state the model's true sizes, not the drawn ones.
	require.ElementsMatch(t, []string{"BLOCK", "SCALE 1:2", "20", "30", "20", "10"}, values)
	for _, tx := range texts {
		switch tx.value {
		case "BLOCK", "SCALE 1:2":
			require.Equal(t, export.DXFTextLayer, tx.layer)
			require.InDelta(t, 5, tx.height, 1e-9)
		default:
			require.InDelta(t, 3.5, tx.height, 1e-9)
		}
	}
	require.NotEmpty(t, byLayer[export.DXFDimensionLayer])
}

func TestDXFSheetWithoutDimensionsDrawsOnlyBorderTextAndViews(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	var out bytes.Buffer
	sheet := a4Sheet(false)
	sheet.Scale = units.Scalar(2)
	require.NoError(t, export.DXFSheet(t.Context(), &out, body, units.Millimeters(0.1), sheet))
	lines, _ := readDXFLines(t, out.String())
	for _, l := range lines {
		require.NotEqual(t, export.DXFDimensionLayer, l.layer)
	}
	var values []string
	for _, tx := range readDXFTexts(t, out.String()) {
		values = append(values, tx.value)
	}
	require.Equal(t, []string{"BLOCK", "SCALE 2:1"}, values)
	// At 2:1 the top view is 40 by 60 and the front view 40 by 20.
	var top []dxfLine
	for _, l := range lines {
		if l.layer == export.DXFVisibleLayer && l.a[0] < 150 && l.b[0] < 150 {
			top = append(top, l)
		}
	}
	minX, maxX, minY, maxY := sheetBounds(top)
	require.InDelta(t, 40, maxX-minX, 1e-9)
	require.InDelta(t, 60, maxY-minY, 1e-9)
}

func TestDXFSheetIsDeterministic(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	var a, b bytes.Buffer
	require.NoError(t, export.DXFSheet(t.Context(), &a, body, units.Millimeters(0.1), a4Sheet(true)))
	require.NoError(t, export.DXFSheet(t.Context(), &b, body, units.Millimeters(0.1), a4Sheet(true)))
	require.Equal(t, a.String(), b.String())
}

func TestDXFSheetRefusals(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	tol := units.Millimeters(0.1)
	with := func(edit func(*export.DXFPaper)) error {
		sheet := a4Sheet(true)
		edit(&sheet)
		var out bytes.Buffer
		err := export.DXFSheet(t.Context(), &out, body, tol, sheet)
		require.Zero(t, out.Len(), "a refused sheet writes nothing")
		return err
	}
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Views = nil }), decad.ErrDegenerate)
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Scale = units.Scalar(0) }), decad.ErrDegenerate)
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Scale = units.Scalar(-1) }), decad.ErrNegativeMagnitude)
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Scale = units.Millimeters(1) }), decad.ErrUnitKind)
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Width = units.Millimeters(0) }), decad.ErrDegenerate)
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Height = units.Scalar(210) }), decad.ErrUnitKind)
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Margin = units.Millimeters(-1) }), decad.ErrNegativeMagnitude)
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Title = "café" }), decad.ErrDegenerate)
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Title = "two\nlines" }), decad.ErrDegenerate)
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Gap = units.Millimeters(5) }), decad.ErrDegenerate,
		"two dimensioned views need 12 mm between them")
	require.ErrorIs(t, with(func(p *export.DXFPaper) { p.Scale = units.Scalar(10) }), decad.ErrDegenerate, "too big for the paper")
	var out bytes.Buffer
	require.ErrorIs(t, export.DXFSheet(t.Context(), nil, body, tol, a4Sheet(true)), decad.ErrDegenerate)
	require.ErrorIs(t, export.DXFSheet(t.Context(), &out, nil, tol, a4Sheet(true)), decad.ErrDegenerate)
}

func TestDXFSheetWritesTheTitleBlockFields(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	paper := a4Sheet(false)
	paper.Fields = []export.DXFField{{"PART", "BK-100"}, {"AUTHOR", "A. N. OTHER"}, {"REV", "C"}}
	var out bytes.Buffer
	require.NoError(t, export.DXFSheet(t.Context(), &out, body, units.Millimeters(0.1), paper))
	byValue := map[string]dxfText{}
	for _, tx := range readDXFTexts(t, out.String()) {
		byValue[tx.value] = tx
	}
	// The fields stack down the right of the title, small, each four millimetres under the last.
	part, author, rev := byValue["PART: BK-100"], byValue["AUTHOR: A. N. OTHER"], byValue["REV: C"]
	require.Equal(t, export.DXFTextLayer, part.layer)
	require.InDelta(t, 2.5, part.height, 1e-9)
	require.InDelta(t, part.at[0], author.at[0], 1e-9)
	require.InDelta(t, part.at[1]-4, author.at[1], 1e-9)
	require.InDelta(t, part.at[1]-8, rev.at[1], 1e-9)
	// They sit inside the title block: above its bottom line and below its top, left of the scale.
	require.Greater(t, part.at[1], 10.0)
	require.Less(t, part.at[1], 10+14.0)
	require.Less(t, part.at[0], 297-10-60.0)
	// No fields, no extra text.
	var plain bytes.Buffer
	require.NoError(t, export.DXFSheet(t.Context(), &plain, body, units.Millimeters(0.1), a4Sheet(false)))
	require.Len(t, readDXFTexts(t, plain.String()), 2)
}

func TestDXFSheetTitleBlockFieldRefusals(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	with := func(fields ...export.DXFField) error {
		paper := a4Sheet(false)
		paper.Fields = fields
		var out bytes.Buffer
		err := export.DXFSheet(t.Context(), &out, body, units.Millimeters(0.1), paper)
		require.Zero(t, out.Len(), "a refused sheet writes nothing")
		return err
	}
	require.ErrorIs(t, with(export.DXFField{"A", "1"}, export.DXFField{"B", "2"}, export.DXFField{"C", "3"}, export.DXFField{"D", "4"},
		export.DXFField{"E", "5"}, export.DXFField{"F", "6"}, export.DXFField{"G", "7"}), decad.ErrDegenerate)
	// With more than three fields the block has two columns and a field holds 24 characters at most.
	long := export.DXFField{"DESCRIPTION", "TWELVE CHARS"}
	require.ErrorIs(t, with(long, export.DXFField{"B", "2"}, export.DXFField{"C", "3"}, export.DXFField{"D", "4"}), decad.ErrDegenerate)
	// Three fields stay in one column, where the limit does not apply.
	paper := a4Sheet(false)
	paper.Fields = []export.DXFField{{"DESCRIPTION", "TWELVE CHARS"}, {"B", "2"}, {"C", "3"}}
	var out bytes.Buffer
	require.NoError(t, export.DXFSheet(t.Context(), &out, body, units.Millimeters(0.1), paper))
	require.ErrorIs(t, with(export.DXFField{"", "1"}), decad.ErrDegenerate)
	require.ErrorIs(t, with(export.DXFField{"NAME", "café"}), decad.ErrDegenerate)
	require.ErrorIs(t, with(export.DXFField{"NA\nME", "x"}), decad.ErrDegenerate)
}

func TestDXFSheetSetsMoreThanThreeFieldsInTwoColumns(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	paper := a4Sheet(false)
	paper.Fields = []export.DXFField{
		{"PART", "BK-100"}, {"AUTHOR", "A. N. OTHER"}, {"REV", "C"},
		{"DATE", "2026-10-28"}, {"MATERIAL", "AL 6061"}, {"SHEET", "1 OF 2"},
	}
	var out bytes.Buffer
	require.NoError(t, export.DXFSheet(t.Context(), &out, body, units.Millimeters(0.1), paper))
	at := map[string]dxfText{}
	for _, tx := range readDXFTexts(t, out.String()) {
		at[tx.value] = tx
	}
	part, rev, date, sheet := at["PART: BK-100"], at["REV: C"], at["DATE: 2026-10-28"], at["SHEET: 1 OF 2"]
	// The first three stack in the first column, the next three start again at the top of a second to its right.
	require.InDelta(t, part.at[0], rev.at[0], 1e-9)
	require.InDelta(t, part.at[1]-8, rev.at[1], 1e-9)
	require.InDelta(t, part.at[1], date.at[1], 1e-9)
	require.InDelta(t, part.at[1]-8, sheet.at[1], 1e-9)
	require.InDelta(t, date.at[0], sheet.at[0], 1e-9)
	require.Greater(t, date.at[0]-part.at[0], 30.0)
	// Both columns stay left of the scale's divider, 60 mm in from the right border, and the second leaves room.
	require.Less(t, date.at[0]+maxWideDXFFieldWidth, 297-10-60.0)
	require.Len(t, readDXFTexts(t, out.String()), 8)
}

// maxWideDXFFieldWidth is the width in millimetres of a 24-character field of 2.5 mm text, about 0.6 of its height
// a letter.
const maxWideDXFFieldWidth = 24 * 0.6 * 2.5
