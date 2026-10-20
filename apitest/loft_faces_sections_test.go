package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

// splitSlabCaps splits one point-loft tooth twice and returns the first
// slab's start cap and the second slab's end cap, the two sections a loft
// through that tooth needs.
func splitSlabCaps(t *testing.T, doc *decad.Document) (*decad.Face, *decad.Face) {
	t.Helper()
	inner := pointLoftRadialTab(t, doc)
	span := pointLoftSliceSpan()
	var kept []*decad.Body
	for _, k := range []float64{1, 2} {
		sheet := pointLoftSliceSheet(t, doc, span*k/6, 30)
		parts, err := decad.Split(t.Context(), inner, sheet)
		require.NoError(t, err)
		require.Len(t, parts, 2)
		inner = parts[0]
		kept = append(kept, parts[1])
	}
	return pointSlabCap(t, kept[1], decad.CapStart(kept[1])), pointSlabCap(t, kept[0], decad.CapEnd(kept[0]))
}

func TestLoftFacesRefusesUnauthenticatedSections(t *testing.T) {
	doc := decad.New()
	_, end := splitSlabCaps(t, doc)
	otherStart, _ := splitSlabCaps(t, doc)
	foreignStart, _ := splitSlabCaps(t, decad.New())
	tooth := pointLoftRadialTab(t, doc)

	tests := []struct {
		name     string
		sections []*decad.Face
		err      error
		msg      string
	}{
		{"nil section", []*decad.Face{nil, end}, decad.ErrDegenerate, "is not a body face"},
		{"face of an unsplit point loft", []*decad.Face{tooth.Faces()[0], end}, decad.ErrUnsupported,
			"no authenticated Split cap"},
		{"sections of different sources", []*decad.Face{otherStart, end}, decad.ErrUnsupported,
			"different point-loft sources"},
		{"section of another document", []*decad.Face{foreignStart, end}, decad.ErrForeignBody, ""},
	}
	for _, tc := range tests {
		before := doc.Bodies()
		_, err := decad.LoftFaces(t.Context(), doc, tc.sections...)
		require.ErrorIs(t, err, tc.err, tc.name)
		if tc.msg != "" {
			require.ErrorContains(t, err, tc.msg, tc.name)
		}
		require.Equal(t, before, doc.Bodies(), tc.name)
	}
}
