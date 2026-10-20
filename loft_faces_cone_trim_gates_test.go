package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/loftfacestrim"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// editToe replaces a copy of the cut's toe record and cap record, so a test
// can break one fact of the proof at a time.
func editToe(c tangentTrimChain, edit func(*facetedrecord.LoftFacesToeRecord, *facetedrecord.LoftFacesCapRecord)) {
	p := c.cut.n.Payload.(facetedPayload)
	toe := *p.LoftFacesToe
	capRecord := *toe.Cap
	edit(&toe, &capRecord)
	toe.Cap = &capRecord
	p.LoftFacesToe = &toe
	c.cut.n.Payload = p
}

// TestConeHeelBooleanGates breaks one fact of the toe-cut tooth at a time.
// The heel Intersect declines every broken fact without error.
func TestConeHeelBooleanGates(t *testing.T) {
	t.Parallel()

	run := func(c tangentTrimChain, op meshbool.OperationKind, tool *Body) (*topo.Body, bool, error) {
		return loftfacestrim.ConeHeelBoolean(t.Context(), op, envOf(c.cut.n.Doc, c.cut.n.Doc.NextProducer),
			pairMeshes{}, &c.cut.n, &tool.n)
	}

	t.Run("accepts the real chain", func(t *testing.T) {
		t.Parallel()
		c := buildTangentTrimChain(t)
		got, matched, err := run(c, meshbool.OpIntersect, c.heelTool)
		require.NoError(t, err)
		require.True(t, matched)
		require.NotNil(t, got.Payload.(facetedPayload).LoftFacesTrim)
	})

	declines := []struct {
		name string
		edit func(t *testing.T, c *tangentTrimChain, op *meshbool.OperationKind, tool **Body)
	}{
		{"cut", func(_ *testing.T, _ *tangentTrimChain, op *meshbool.OperationKind, _ **Body) { *op = meshbool.OpCut }},
		{"union", func(_ *testing.T, _ *tangentTrimChain, op *meshbool.OperationKind, _ **Body) { *op = meshbool.OpUnion }},
		{"unfaceted target", func(_ *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, _ **Body) {
			c.cut.n.Payload = nil
		}},
		{"no toe record", func(_ *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, _ **Body) {
			p := c.cut.n.Payload.(facetedPayload)
			p.LoftFacesToe = nil
			c.cut.n.Payload = p
		}},
		{"short cap record", func(_ *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, _ **Body) {
			editToe(*c, func(_ *facetedrecord.LoftFacesToeRecord, r *facetedrecord.LoftFacesCapRecord) {
				r.Maps = r.Maps[:1]
			})
		}},
		{"placed target", func(_ *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, _ **Body) {
			p := c.cut.n.Payload.(facetedPayload)
			p.Xform, _ = r3.Translation(r3.Vec{X: 1})
			c.cut.n.Payload = p
		}},
		{"no cap face", func(_ *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, _ **Body) {
			editToe(*c, func(toe *facetedrecord.LoftFacesToeRecord, _ *facetedrecord.LoftFacesCapRecord) { toe.CapFace = nil })
		}},
		{"cap face of another body", func(_ *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, _ **Body) {
			editToe(*c, func(toe *facetedrecord.LoftFacesToeRecord, _ *facetedrecord.LoftFacesCapRecord) {
				toe.CapFace = faceNodes(c.heelTool.Faces())[0]
			})
		}},
		{"cap face not in the body", func(_ *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, _ **Body) {
			editToe(*c, func(toe *facetedrecord.LoftFacesToeRecord, _ *facetedrecord.LoftFacesCapRecord) {
				toe.CapFace = &topo.Face{Body: &c.cut.n}
			})
		}},
		{"toe cap face not in the body", func(_ *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, _ **Body) {
			editToe(*c, func(toe *facetedrecord.LoftFacesToeRecord, _ *facetedrecord.LoftFacesCapRecord) {
				toe.ToeCapFace = &topo.Face{}
			})
		}},
		{"tool is no cone", func(_ *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, tool **Body) {
			*tool = c.loft
		}},
		{"cone of another far plane", func(t *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, tool **Body) {
			*tool = c.f.coneAt(t, -5, 8.625, 8)
		}},
		{"heel cone before the toe cone", func(t *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, tool **Body) {
			*tool = c.f.coneAt(t, -4, 8.625, 7)
		}},
		{"cone off the tangent apex", func(t *testing.T, c *tangentTrimChain, _ *meshbool.OperationKind, tool **Body) {
			*tool = c.f.coneAt(t, -4, 16, math.Nextafter(8, 9))
		}},
	}
	for _, tc := range declines {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			c := buildTangentTrimChain(t)
			op, tool := meshbool.OpIntersect, c.heelTool
			tc.edit(t, &c, &op, &tool)
			_, matched, err := run(c, op, tool)
			require.NoError(t, err)
			require.False(t, matched)
		})
	}
}

// TestRecordConeToeGates breaks one fact of the cut at a time. A broken fact
// leaves the result without a toe record and without error.
func TestRecordConeToeGates(t *testing.T) {
	t.Parallel()

	run := func(t *testing.T, c tangentTrimChain, source, tool *topo.Body) (*facetedrecord.LoftFacesToeRecord, error) {
		t.Helper()
		p := c.cut.n.Payload.(facetedPayload)
		p.LoftFacesToe = nil
		c.cut.n.Payload = p
		err := loftfacestrim.RecordConeToe(t.Context(), source, tool, &c.cut.n)
		return c.cut.n.Payload.(facetedPayload).LoftFacesToe, err
	}

	t.Run("records the toe of the real cut", func(t *testing.T) {
		t.Parallel()
		c := buildTangentTrimChain(t)
		toe, err := run(t, c, &c.loft.n, &c.toeTool.n)
		require.NoError(t, err)
		require.NotNil(t, toe)
		require.Same(t, &c.toeTool.n, toe.ToeTool)
	})

	declines := []struct {
		name string
		edit func(t *testing.T, c *tangentTrimChain, source, tool **topo.Body)
	}{
		{"unfaceted source", func(_ *testing.T, c *tangentTrimChain, source, _ **topo.Body) {
			*source = &topo.Body{}
		}},
		{"source is no primitive", func(_ *testing.T, c *tangentTrimChain, _, _ **topo.Body) {
			p := c.loft.n.Payload.(facetedPayload)
			p.LoftFacesPrimitive = false
			c.loft.n.Payload = p
		}},
		{"source cap record short", func(_ *testing.T, c *tangentTrimChain, _, _ **topo.Body) {
			p := c.loft.n.Payload.(facetedPayload)
			record := *p.LoftFacesCap
			record.Maps = record.Maps[:1]
			p.LoftFacesCap = &record
			c.loft.n.Payload = p
		}},
		{"source placed", func(_ *testing.T, c *tangentTrimChain, _, _ **topo.Body) {
			p := c.loft.n.Payload.(facetedPayload)
			p.Xform, _ = r3.Translation(r3.Vec{X: 1})
			c.loft.n.Payload = p
		}},
		{"tool is no cone", func(_ *testing.T, c *tangentTrimChain, _, tool **topo.Body) { *tool = &c.loft.n }},
		{"cone far plane above the source", func(t *testing.T, c *tangentTrimChain, _, tool **topo.Body) {
			*tool = &c.f.coneAt(t, 50, 8.625, 57.5).n
		}},
		{"cone past the source cap", func(t *testing.T, c *tangentTrimChain, _, tool **topo.Body) {
			*tool = &c.f.coneAt(t, -4, 8.625, -3).n
		}},
	}
	for _, tc := range declines {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			c := buildTangentTrimChain(t)
			source, tool := &c.loft.n, &c.toeTool.n
			tc.edit(t, &c, &source, &tool)
			toe, err := run(t, c, source, tool)
			require.NoError(t, err)
			require.Nil(t, toe)
		})
	}
}

// TestTangentHeelMeshesClearsResult states the held-only Gate E5 verdicts the
// real chain does not reach.
func TestTangentHeelMeshesClearsResult(t *testing.T) {
	t.Parallel()

	// ranMeshes is the recorder after the extended Boolean read the real chain.
	ranMeshes := func(t *testing.T) *loftfacestrim.TangentHeelMeshes {
		t.Helper()
		c := buildTangentTrimChain(t)
		final, _, box, heel := tangentGateInputs(t, c)
		side, ok := loftface.ConeOnLoftSide(final, c.cut.n.Payload.(facetedPayload).LoftFacesToe.Cap.Maps[len(c.loftRecord.Maps)-2], box,
			heel.Apex, heel.Slope, heel.Far)
		require.True(t, ok)
		plane, ok := loftface.NewTangentPlane(final, side)
		require.True(t, ok)
		toe := c.cut.n.Payload.(facetedPayload).LoftFacesToe
		for scale := 4.0; scale <= 32; scale *= 2 {
			m := &loftfacestrim.TangentHeelMeshes{Base: pairMeshes{}, Target: &c.cut.n, Toe: toe, Plane: plane}
			if _, err := evaluateBooleanMeshes(t.Context(), meshbool.OpIntersect, c.cut, c.heelTool, m, scale); err == nil {
				return m
			}
		}
		require.FailNow(t, "the extended Boolean never succeeded")
		return nil
	}

	t.Run("a result vertex beyond the slab", func(t *testing.T) {
		t.Parallel()
		m := ranMeshes(t)
		require.NoError(t, m.ClearsResult(t.Context(), nil))
		err := m.ClearsResult(t.Context(), []r3.Vec{{X: 1e6, Y: 1e6, Z: 1e6}, {X: -1e6, Y: -1e6, Z: -1e6}})
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "beyond the end-cap slab")
	})

	t.Run("a cone with no vertices", func(t *testing.T) {
		t.Parallel()
		m := ranMeshes(t)
		m.ConeVerts = nil
		err := m.ClearsResult(t.Context(), nil)
		require.Error(t, err)
	})

	t.Run("records no meshes", func(t *testing.T) {
		t.Parallel()
		m := &loftfacestrim.TangentHeelMeshes{}
		err := m.ClearsResult(t.Context(), nil)
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "recorded no operand meshes")
	})
}
