package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPrismPatternSourceProvesCircularJoinOfRotatedFittedSections(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	root := math.Sqrt(10)
	blank := bornBody(&Body{n: topo.Body{
		Payload: prismrecord.Payload{
			Frame: frame, Xform: r3.Identity(), Z1: 2,
			Profile: profileRecord{Outer: loopRecord{Segments: []curveSegment{
				circleSeg{Center: planePoint{}, Radius: units.Millimeters(root), TStart: 0, TEnd: 1},
			}}},
		},
	}})
	section := prismrecord.Payload{
		Frame: frame, Xform: r3.Identity(), Z1: 2,
		Profile: profileRecord{Outer: loopRecord{Segments: []curveSegment{
			arcSeg{Center: planePoint{}, Start: planePoint{U: 3, V: -1},
				End: planePoint{U: 3, V: 1}, TStart: 0, TEnd: 1},
			lineSeg{Start: planePoint{U: 3, V: 1}, End: planePoint{U: 4, V: 1}, TStart: 0, TEnd: 1},
			fitSplineSeg{Fit: point2ToRecordSlice([]planePoint{{U: 4, V: 1}, {U: 4.5, V: 1.5}, {U: 5, V: 2}}),
				TStart: 0, TEnd: 1},
			arcSeg{Center: planePoint{}, Start: planePoint{U: 5, V: -2},
				End: planePoint{U: 5, V: 2}, TStart: 1, TEnd: 0},
			fitSplineSeg{Fit: point2ToRecordSlice([]planePoint{{U: 5, V: -2}, {U: 4.5, V: -1.5}, {U: 4, V: -1}}),
				TStart: 0, TEnd: 1},
			lineSeg{Start: planePoint{U: 4, V: -1}, End: planePoint{U: 3, V: -1}, TStart: 0, TEnd: 1},
		}}},
	}
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/2))
	require.NoError(t, err)
	copySection := section
	copySection.Xform = turn
	first := bornBody(&Body{n: topo.Body{Payload: section}})
	copyBody := bornBody(&Body{n: topo.Body{Payload: copySection}})
	operands := []*Body{blank, first, copyBody}

	source, matched, err := prismPatternSourceOf(t.Context(), operands)
	require.NoError(t, err)
	require.True(t, matched)
	require.Len(t, source.sections, 2)
	require.Less(t, source.rootGap, 1e-12)

	// Rebuilding a similar section is not a Copy certificate.
	separate := copySection
	separate.Profile.Outer.Segments = append([]curveSegment(nil), copySection.Profile.Outer.Segments...)
	operands[2] = bornBody(&Body{n: topo.Body{Payload: separate}})
	_, matched, err = prismPatternSourceOf(t.Context(), operands)
	require.True(t, matched)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "not a radial copy")
}
