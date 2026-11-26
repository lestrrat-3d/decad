package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// The copy helpers deep-copy the selector a sealed argument holds, in every
// position the argument can hold one: directly, as a non-nil pointer, and as a
// side of a two-sided extent (agent-docs/command-api-design.md rule A3). A
// change to the caller's query after the call reaches only the original.
func TestFeatureCopyHelpersDeepCopyTheirSelectors(t *testing.T) {
	t.Parallel()
	t.Run("ToFace", func(t *testing.T) {
		t.Parallel()
		q := Faces(Planar())
		requireIndependentCopies(t, q, map[string]*FaceQuery{
			"value":                copyFaceOf(t, copyExtent(ToFace{Face: q})),
			"pointer":              copyFaceOf(t, copyExtent(&ToFace{Face: q})),
			"side":                 copyFaceOf(t, copyExtent(TwoSided{One: ToFace{Face: q}, Two: DistanceSide{}})),
			"pointer to two sided": copyFaceOf(t, copyExtent(&TwoSided{One: DistanceSide{}, Two: &ToFace{Face: q}})),
		})
	})
	t.Run("ToFaceAngular", func(t *testing.T) {
		t.Parallel()
		q := Faces(Planar())
		requireIndependentCopies(t, q, map[string]*FaceQuery{
			"value":                copyFaceOf(t, copyAngularExtent(ToFaceAngular{Face: q})),
			"pointer":              copyFaceOf(t, copyAngularExtent(&ToFaceAngular{Face: q})),
			"side":                 copyFaceOf(t, copyAngularExtent(TwoSidedAngle{One: ToFaceAngular{Face: q}, Two: AngleSide{}})),
			"pointer to two sided": copyFaceOf(t, copyAngularExtent(&TwoSidedAngle{One: AngleSide{}, Two: &ToFaceAngular{Face: q}})),
		})
	})
	t.Run("EdgeAxis", func(t *testing.T) {
		t.Parallel()
		q := Edges()
		before := q.String()
		copies := map[string]*EdgeQuery{}
		for name, a := range map[string]Axis{"value": EdgeAxis{Edge: q}, "pointer": &EdgeAxis{Edge: q}} {
			var got EdgeSelector
			switch v := copyAxis(a).(type) {
			case EdgeAxis:
				got = v.Edge
			case *EdgeAxis:
				got = v.Edge
			}
			require.NotSame(t, q, got, name)
			copies[name] = got.(*EdgeQuery)
		}
		require.NotEqual(t, before, q.Exactly(3).String())
		for name, cp := range copies {
			require.Equal(t, before, cp.String(), name)
		}
	})
	t.Run("nil and selector-free values pass through", func(t *testing.T) {
		t.Parallel()
		require.Nil(t, copyExtent(nil))
		require.Equal(t, Extent((*ToFace)(nil)), copyExtent((*ToFace)(nil)))
		require.Equal(t, Extent(Distance{}), copyExtent(Distance{}))
		require.Nil(t, copyAngularExtent(nil))
		require.Equal(t, AngularExtent(FullRevolution{}), copyAngularExtent(FullRevolution{}))
		require.Nil(t, copyAxis(nil))
		require.Equal(t, Axis(SketchLine{}), copyAxis(SketchLine{}))
	})
}

// requireIndependentCopies requires every copy to be a different query from q
// that keeps q's rendering after the caller changes q.
func requireIndependentCopies(t *testing.T, q *FaceQuery, copies map[string]*FaceQuery) {
	t.Helper()
	before := q.String()
	for name, cp := range copies {
		require.NotSame(t, q, cp, name)
	}
	require.NotEqual(t, before, q.Exactly(3).String())
	for name, cp := range copies {
		require.Equal(t, before, cp.String(), name)
	}
}

// copyFaceOf returns the face query held by an extent or an angular extent,
// directly, as a non-nil pointer or as the first face stop among its sides.
func copyFaceOf(t *testing.T, extent any) *FaceQuery {
	t.Helper()
	var stops []any
	switch v := extent.(type) {
	case ToFace:
		stops = []any{v}
	case *ToFace:
		stops = []any{v}
	case TwoSided:
		stops = []any{v.One, v.Two}
	case *TwoSided:
		stops = []any{v.One, v.Two}
	case ToFaceAngular:
		stops = []any{v}
	case *ToFaceAngular:
		stops = []any{v}
	case TwoSidedAngle:
		stops = []any{v.One, v.Two}
	case *TwoSidedAngle:
		stops = []any{v.One, v.Two}
	}
	for _, stop := range stops {
		switch v := stop.(type) {
		case ToFace:
			return v.Face.(*FaceQuery)
		case *ToFace:
			return v.Face.(*FaceQuery)
		case ToFaceAngular:
			return v.Face.(*FaceQuery)
		case *ToFaceAngular:
			return v.Face.(*FaceQuery)
		}
	}
	require.FailNow(t, "no face stop in the extent")
	return nil
}
