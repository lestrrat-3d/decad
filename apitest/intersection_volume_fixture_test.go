package apitest_test

import (
	"errors"
	"fmt"
	"hash/fnv"
	"math"
	"testing"

	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// volumeCase builds one operand pair in a fresh document. Calling make twice gives two twin documents
// whose pairs are built the same way.
type volumeCase struct {
	name string
	// outcome is what Intersect returns for the pair: "ok", "empty", "empty-shift" or "unsupported".
	outcome string
	make    func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body)
}

func intersectionVolumeCases() []volumeCase {
	return []volumeCase{
		{"overlapping prisms", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10), boxBodyAtZ(t, doc, 5, 5, 15, 15, 5, 10)
		}},
		{"coplanar co-directional prisms", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, boxBody(t, doc, 0, 0, 10, 10, 10), boxBody(t, doc, 5, 5, 15, 15, 10)
		}},
		{"far-apart boxes", "empty", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, boxBody(t, doc, 0, 0, 10, 10, 10), boxBody(t, doc, 100, 100, 110, 110, 10)
		}},
		{"face-touching boxes, gap shift", "empty-shift", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			a, b := gapBoxes(t, 0.5, 0, 10, 10, 10, 10)
			return nil, a, b
		}},
		{"face-touching boxes, exact only", "unsupported", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			a, b := gapBoxes(t, 0.5, 0, 10, 10, 10, 10, decad.ExactOnly())
			return nil, a, b
		}},
		{"cylinder through a box", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, roundTrip(t, boxBodyAtZ(t, doc, 0, 0, 20, 20, 0, 10)), circlePrismAtZ(t, doc, 10, 10, 4, -2, 14)
		}},
		{"orthogonal mesh solids", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			a := roundTrip(t, boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10))
			b := roundTrip(t, boxBodyAtZ(t, doc, 4, 4, 14, 14, 3, 10))
			return doc, a, b
		}},
		{"regroup after a mesh cut", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			sc := newRetentionScene()
			box, t1, c := sc.roundedBox(t), sc.tool(t, 20), regroupSlabC(t, sc)
			first, err := decad.Cut(box, t1).Do(t.Context())
			require.NoError(t, err)
			return sc.doc, first, c
		}},
		// The grid route: SolidFromMesh blocks every face of which squares to a world axis take the exact
		// orthogonal Boolean, which commits with a single producer.
		{"grid blocks overlap", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10)),
				blockIn(t, doc, r3.NewVec(4, 3, 2), r3.NewVec(14, 13, 12))
		}},
		{"grid block and extruded box", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10)),
				boxBodyAtZ(t, doc, 4, 3, 14, 13, 2, 10)
		}},
		{"grid blocks touching", "empty", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10)),
				blockIn(t, doc, r3.NewVec(10, 0, 0), r3.NewVec(20, 10, 10))
		}},
		// A tool wall crosses a tapered hole's wall, whose facets hold a rim bound coarser than the pair's chord
		// tolerance: the held operand cannot decide the contact.
		{"tool wall on a coarse held rim", "unsupported", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			sc := newRetentionScene()
			built, err := decad.Cut(sc.roundedBox(t), sc.taperedTool(t)).Do(t.Context())
			require.NoError(t, err)
			// A Copy ends the Boolean chain, so the refusal is the sequential gate.
			first, err := decad.Copy(built).Do(t.Context())
			require.NoError(t, err)
			return sc.doc, first, slabYTool(t, sc, 22, 3)
		}},
	}
}

// revolveRetryCases are the pairs whose revolved operand the evaluator coarsens past its work ceilings and
// retries: a half torus arc and a radial stem, in either operand order. Each costs seconds under -race, so
// only the tests that need the retry route use them.
func revolveRetryCases() []volumeCase {
	return []volumeCase{
		{"torus arc and stem", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, torusArcBody(t, doc, math.Pi), radialStemBody(t, doc)
		}},
		{"stem and torus arc", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, radialStemBody(t, doc), torusArcBody(t, doc, math.Pi)
		}},
	}
}

// allIntersectionVolumeCases is the common cases and the revolve retry cases.
func allIntersectionVolumeCases() []volumeCase {
	return append(intersectionVolumeCases(), revolveRetryCases()...)
}

// build makes the case's pair and the document that holds it.
func (c volumeCase) build(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	doc, a, b := c.make(t)
	if doc == nil {
		doc = a.Document()
	}
	return doc, a, b
}

// requireSameOutcome requires the query's outcome and Intersect's to agree: nil-ness, sentinels, the boolean
// error Code, message and gap shift, and the volume's value, exactness and bound by bits.
func requireSameOutcome(t *testing.T, queried decad.Measurement, qerr error, got *decad.Body, ierr error) {
	t.Helper()
	if ierr != nil {
		require.Error(t, qerr)
		for _, sentinel := range []error{decad.ErrBooleanFailed, decad.ErrUnsupported, decad.ErrDegenerate} {
			require.Equal(t, errors.Is(ierr, sentinel), errors.Is(qerr, sentinel), "sentinel %v", sentinel)
		}
		var ib, qb *decad.BooleanError
		require.Equal(t, errors.As(ierr, &ib), errors.As(qerr, &qb))
		if ib != nil {
			require.Equal(t, ib.Code, qb.Code)
			require.Equal(t, ib.Error(), qb.Error())
			require.Equal(t, ib.OperationDiagnostics(), qb.OperationDiagnostics())
		}
		return
	}
	require.NoError(t, qerr)
	want, err := got.Volume()
	require.NoError(t, err)
	requireSameMeasurement(t, want, queried)
}

func requireSameMeasurement(t *testing.T, want, got decad.Measurement) {
	t.Helper()
	require.Equal(t, want.Exactness, got.Exactness)
	require.Equal(t, math.Float64bits(want.Value.Base()), math.Float64bits(got.Value.Base()))
	require.Equal(t, math.Float64bits(want.Bound.Base()), math.Float64bits(got.Bound.Base()))
}

// requireOutcomeKind requires err to be the outcome a case names.
func requireOutcomeKind(t *testing.T, want string, err error) {
	t.Helper()
	var be *decad.BooleanError
	switch want {
	case "ok":
		require.NoError(t, err)
	case "empty":
		require.ErrorAs(t, err, &be)
		require.Equal(t, decad.BooleanEmpty, be.Code)
	case "empty-shift":
		require.ErrorAs(t, err, &be)
		require.Equal(t, decad.BooleanEmpty, be.Code)
		shifted := false
		for _, d := range be.OperationDiagnostics() {
			shifted = shifted || d.Change != nil
		}
		require.True(t, shifted, "the empty result reports the gap shift")
	case "unsupported":
		require.ErrorIs(t, err, decad.ErrUnsupported)
	default:
		require.Failf(t, "bad outcome", "%q", want)
	}
}

// outcomeKey summarizes a query or Intersect outcome as a comparable string: the boolean error code and
// message, or the volume's value, bound and exactness by bits.
func outcomeKey(m decad.Measurement, err error) string {
	if err == nil {
		return fmt.Sprintf("ok:%x:%x:%v", math.Float64bits(m.Value.Base()), math.Float64bits(m.Bound.Base()), m.Exactness)
	}
	if be, ok := errors.AsType[*decad.BooleanError](err); ok {
		return fmt.Sprintf("boolean-error:%v:%s", be.Code, be.Error())
	}
	return "error:" + err.Error()
}

// meshHash fingerprints b's tessellation by the bits of its vertices and its triangles.
func meshHash(t *testing.T, b *decad.Body) string {
	t.Helper()
	m, err := b.Tessellate(t.Context(), units.Millimeters(0.2))
	if err != nil {
		return "tessellate-error:" + err.Error()
	}
	h := fnv.New64a()
	for _, v := range m.Vertices() {
		fmt.Fprintf(h, "%x,%x,%x;", math.Float64bits(v.X), math.Float64bits(v.Y), math.Float64bits(v.Z))
	}
	fmt.Fprintf(h, "%v", m.Triangles())
	return fmt.Sprintf("%d/%d/%x", len(m.Vertices()), len(m.Triangles()), h.Sum64())
}

// laterOperations runs Intersect on a and b, then a Union with a third body and a Cut by a bore, and describes
// each outcome. A refused Intersect leaves the pair live, so a Cut of the pair stands in for the rest.
func laterOperations(t *testing.T, d *decad.Document, a, b *decad.Body) []string {
	t.Helper()
	describe := func(label string, body *decad.Body, err error) []string {
		if err != nil {
			return []string{label + ":" + outcomeKey(decad.Measurement{}, err)}
		}
		v, verr := body.Volume()
		require.NoError(t, verr)
		bb, berr := body.Bounds()
		require.NoError(t, berr)
		var origins []string
		for _, f := range body.Faces() {
			origins = append(origins, fmt.Sprint(f.Origins()))
		}
		return []string{label + ":" + outcomeKey(v, nil), fmt.Sprintf("bounds=%v", bb),
			fmt.Sprintf("faces=%d", len(body.Faces())), meshHash(t, body), fmt.Sprint(origins)}
	}
	ops := func() string { return fmt.Sprintf("ops=%d", len(d.OperationHistory().Operations())) }

	r, err := decad.Intersect(a, b).Do(t.Context())
	out := describe("intersect", r, err)
	if err != nil {
		cut, cerr := decad.Cut(a, b).Do(t.Context())
		return append(append(out, describe("cut", cut, cerr)...), ops())
	}
	third := boxBodyAtZ(t, d, 3, 3, 8, 8, 2, 20)
	u, err := decad.Union(r, third).Do(t.Context())
	out = append(out, describe("union", u, err)...)
	if err != nil {
		return append(out, ops())
	}
	c, err := decad.Cut(u, circlePrismAtZ(t, d, 6, 6, 1.5, -5, 40)).Do(t.Context())
	return append(append(out, describe("cut", c, err)...), ops())
}
