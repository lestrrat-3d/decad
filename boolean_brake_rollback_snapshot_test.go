package decad

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

type brakeRollbackPatch struct {
	ptr   *provenance.SourcePatch
	value provenance.SourcePatch
}

type brakeRollbackCertificate struct {
	snapshot             *booleaneval.ShiftCertificateSnapshot
	budget               *proofbound.ReplayBudget
	result, target, tool *Body
}

func brakeRollbackCertificates(t *testing.T, ops []*Operation) []brakeRollbackCertificate {
	t.Helper()
	var out []brakeRollbackCertificate
	for i, op := range ops {
		inputs, outputs := op.Inputs(), op.Outputs()
		if len(inputs) != 2 || len(outputs) != 1 || outputs[0] == nil {
			continue
		}
		cert, ok := outputs[0].n.PrivateShift.(*booleaneval.ShiftCertificate)
		if !ok {
			continue
		}
		budget := proofbound.NewBrakeReplayBudget()
		snapshot, err := booleaneval.SnapshotShiftCertificate(t.Context(), budget, cert)
		if err != nil {
			t.Fatalf("snapshot source certificate %d: %v", i, err)
		}
		out = append(out, brakeRollbackCertificate{snapshot, budget, outputs[0], inputs[0].Body, inputs[1].Body})
	}
	return out
}

func brakeRollbackCertificatesCheck(t *testing.T, certificates []brakeRollbackCertificate) {
	t.Helper()
	for i, s := range certificates {
		if err := s.snapshot.Revalidate(t.Context(), s.budget, &s.result.n, &s.target.n, &s.tool.n); err != nil {
			t.Fatalf("source certificate %d changed: %v", i, err)
		}
	}
}

type brakeRollbackBody struct {
	body             *Body
	origin           provenance.Ref
	warnings         []opdiagnostic.Record
	allowanceBits    []uint64
	allowancePresent []bool
	payload          any
	transform        r3.Transform
	verts            []r3.Vec
	tris             [][3]int
	src              []int
	bounds           []float64
	faceOf           []int
	groups           []brakeRollbackGroup
	prism            *brakeRollbackPrism
	faceSources      []*provenance.SourcePatch
	faceOrigins      [][]provenance.Ref
	faceUpstream     [][]provenance.Ref
	patches          []brakeRollbackPatch
}

type brakeRollbackGroup struct {
	source            *provenance.SourcePatch
	origins, upstream []provenance.Ref
}

type brakeRollbackPrism struct {
	payload prismrecord.Payload
	loops   [][]momentinput.CurveSegment
	z0, z1  string
}

func brakeRollbackPrismSnapshot(t *testing.T, p prismrecord.Payload) *brakeRollbackPrism {
	t.Helper()
	s := &brakeRollbackPrism{payload: p}
	if z := p.DenotedZ0(); z != nil {
		s.z0 = z.RatString()
	}
	if z := p.DenotedZ1(); z != nil {
		s.z1 = z.RatString()
	}
	loops := append([]momentinput.LoopRecord{p.Profile.Outer}, p.Profile.Holes...)
	if len(loops) > 1024 {
		t.Fatal("prism loop count exceeds test bound")
	}
	for _, loop := range loops {
		if len(loop.Segments) > 1024 {
			t.Fatal("prism segment count exceeds test bound")
		}
		for _, segment := range loop.Segments {
			switch segment.(type) {
			case momentinput.LineSeg, momentinput.CircleSeg, momentinput.ArcSeg:
			default:
				t.Fatalf("unsupported prism test segment %T", segment)
			}
		}
		s.loops = append(s.loops, slices.Clone(loop.Segments))
	}
	return s
}

func brakeRollbackPrismCheck(t *testing.T, p prismrecord.Payload, s *brakeRollbackPrism, body int) {
	t.Helper()
	q := s.payload
	if p.Frame != q.Frame || p.Xform != q.Xform || math.Float64bits(p.Z0) != math.Float64bits(q.Z0) ||
		math.Float64bits(p.Z1) != math.Float64bits(q.Z1) || math.Float64bits(p.Z0Delta) != math.Float64bits(q.Z0Delta) ||
		math.Float64bits(p.Z1Delta) != math.Float64bits(q.Z1Delta) || math.Float64bits(p.SectionDelta) != math.Float64bits(q.SectionDelta) {
		t.Fatalf("body %d prism levels or frame changed", body)
	}
	var z0, z1 string
	if z := p.DenotedZ0(); z != nil {
		z0 = z.RatString()
	}
	if z := p.DenotedZ1(); z != nil {
		z1 = z.RatString()
	}
	if z0 != s.z0 || z1 != s.z1 {
		t.Fatalf("body %d denoted levels changed", body)
	}
	loops := append([]momentinput.LoopRecord{p.Profile.Outer}, p.Profile.Holes...)
	if len(loops) != len(s.loops) {
		t.Fatalf("body %d prism loops changed", body)
	}
	for i, loop := range loops {
		if !slices.Equal(loop.Segments, s.loops[i]) {
			t.Fatalf("body %d prism loop %d changed", body, i)
		}
	}
}

func brakeRollbackSnapshot(t *testing.T, bodies []*Body) []brakeRollbackBody {
	t.Helper()
	out := make([]brakeRollbackBody, 0, len(bodies))
	for _, b := range bodies {
		s := brakeRollbackBody{body: b, origin: b.n.Origin, warnings: slices.Clone(b.n.OperationWarnings), payload: b.n.Payload}
		for _, w := range s.warnings {
			s.allowancePresent = append(s.allowancePresent, w.ChainAllowance != nil)
			var bits uint64
			if w.ChainAllowance != nil {
				bits = math.Float64bits(*w.ChainAllowance)
			}
			s.allowanceBits = append(s.allowanceBits, bits)
		}
		if b.n.Payload != nil {
			s.transform = b.n.Payload.Transform()
		}
		if p, ok := b.n.Payload.(facetedrecord.Payload); ok {
			s.verts, s.tris, s.src, s.bounds = slices.Clone(p.Verts), slices.Clone(p.Tris), slices.Clone(p.Src), slices.Clone(p.VertexBound)
			s.faceOf = slices.Clone(p.FaceOf)
			if len(p.Groups) > brakeSourceNodeLimit {
				t.Fatal("faceted group count exceeds test bound")
			}
			for _, g := range p.Groups {
				s.groups = append(s.groups, brakeRollbackGroup{g.Source, slices.Clone(g.Origins), slices.Clone(g.Upstream)})
				for p, n := g.Source, 0; p != nil; p, n = p.Parent, n+1 {
					if n >= 1024 {
						t.Fatal("group source chain exceeds test bound")
					}
					s.patches = append(s.patches, brakeRollbackPatch{p, *p})
				}
			}
		}
		if p, ok := b.n.Payload.(prismrecord.Payload); ok {
			s.prism = brakeRollbackPrismSnapshot(t, p)
		}
		for _, face := range b.Faces() {
			s.faceSources = append(s.faceSources, face.n.Source)
			s.faceOrigins = append(s.faceOrigins, slices.Clone(face.n.Origins))
			s.faceUpstream = append(s.faceUpstream, slices.Clone(face.n.Upstream))
			for p, n := face.n.Source, 0; p != nil; p, n = p.Parent, n+1 {
				if n >= 1024 {
					t.Fatal("source chain exceeds test bound")
				}
				s.patches = append(s.patches, brakeRollbackPatch{p, *p})
			}
		}
		out = append(out, s)
	}
	return out
}

func brakeRollbackCheck(t *testing.T, snapshots []brakeRollbackBody) {
	t.Helper()
	for i, s := range snapshots {
		b := s.body
		if b.n.Origin != s.origin {
			t.Fatalf("body %d origin changed", i)
		}
		if len(b.n.OperationWarnings) != len(s.warnings) {
			t.Fatalf("body %d warnings changed", i)
		}
		for j, w := range b.n.OperationWarnings {
			if w != s.warnings[j] || (w.ChainAllowance != nil) != s.allowancePresent[j] {
				t.Fatalf("body %d warning %d changed", i, j)
			}
			if w.ChainAllowance != nil && math.Float64bits(*w.ChainAllowance) != s.allowanceBits[j] {
				t.Fatalf("body %d allowance %d changed", i, j)
			}
		}
		if b.n.Payload == nil {
			if s.payload != nil {
				t.Fatalf("body %d payload removed", i)
			}
		} else if b.n.Payload.Transform() != s.transform {
			t.Fatalf("body %d placement changed", i)
		}
		if p, ok := b.n.Payload.(facetedrecord.Payload); ok {
			if !slices.Equal(p.Verts, s.verts) || !slices.Equal(p.Tris, s.tris) || !slices.Equal(p.Src, s.src) || !slices.Equal(p.VertexBound, s.bounds) || !slices.Equal(p.FaceOf, s.faceOf) || len(p.Groups) != len(s.groups) {
				t.Fatalf("body %d payload values changed", i)
			}
			for j, g := range p.Groups {
				if g.Source != s.groups[j].source || !slices.Equal(g.Origins, s.groups[j].origins) || !slices.Equal(g.Upstream, s.groups[j].upstream) {
					t.Fatalf("body %d group %d provenance changed", i, j)
				}
			}
		} else if s.verts != nil {
			t.Fatalf("body %d faceted payload changed", i)
		}
		if s.prism != nil {
			p, ok := b.n.Payload.(prismrecord.Payload)
			if !ok {
				t.Fatalf("body %d prism payload changed", i)
			}
			brakeRollbackPrismCheck(t, p, s.prism, i)
		}
		faces := b.Faces()
		if len(faces) != len(s.faceSources) {
			t.Fatalf("body %d face count changed", i)
		}
		for j, f := range faces {
			if f.n.Source != s.faceSources[j] || !slices.Equal(f.n.Origins, s.faceOrigins[j]) || !slices.Equal(f.n.Upstream, s.faceUpstream[j]) {
				t.Fatalf("body %d face %d source changed", i, j)
			}
		}
		for _, p := range s.patches {
			if *p.ptr != p.value {
				t.Fatalf("body %d source chain changed", i)
			}
		}
	}
}
