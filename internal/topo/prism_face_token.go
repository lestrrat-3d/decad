package topo

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// PrismFaceKind identifies a face at the point its prism builder creates it.
type PrismFaceKind uint8

const (
	PrismFaceUnknown PrismFaceKind = iota
	PrismFaceWall
	PrismFaceCapStart
	PrismFaceCapEnd
)

// PrismFaceToken binds a built face to its body, producer and source segment.
// It records construction identity only; callers must separately seal geometry.
type PrismFaceToken struct {
	face          *Face
	body          *Body
	producer      provenance.Producer
	kind          PrismFaceKind
	loop, segment int
	sealed        bool
	topology      *prismFaceTopologySeal
	origin        provenance.Ref
	surface       surfacegeom.Surface
	cap           *CapRecord
	capValue      CapRecord
	capLevel      *big.Rat
	scalars       [6]uint64
	flags         [5]bool
	centroid      measurement.VecMeasurement
}

// BeginPrismConstruction opens the one-time face-token recording phase.
func (b *Body) BeginPrismConstruction() bool {
	if b == nil || b.prismConstructionState != 0 || b.Payload != nil {
		return false
	}
	b.prismConstructionState = 1
	return true
}

// FinishPrismConstruction closes the phase permanently.
func (b *Body) FinishPrismConstruction() {
	if b != nil && b.prismConstructionState == 1 {
		b.prismConstructionState = 2
	}
}

// RecordPrismFace stamps an otherwise unstamped face during native construction.
func (f *Face) RecordPrismFace(kind PrismFaceKind, loop, segment int) bool {
	if f == nil || f.Body == nil || f.Body.prismConstructionState != 1 || f.prismFaceToken.kind != PrismFaceUnknown {
		return false
	}
	switch kind {
	case PrismFaceWall:
		if loop < 0 || segment < 0 {
			return false
		}
	case PrismFaceCapStart, PrismFaceCapEnd:
		if loop != -1 || segment != -1 {
			return false
		}
	default:
		return false
	}
	f.prismFaceToken = PrismFaceToken{face: f, body: f.Body, producer: f.Body.Origin.Producer, kind: kind, loop: loop, segment: segment}
	return true
}

// SealPrismFace snapshots restricted face values after the prism build finishes.
func (f *Face) SealPrismFace() bool {
	if f == nil || f.Body == nil || f.Body.prismConstructionState != 1 {
		return false
	}
	t := f.prismFaceToken
	if t.face != f || t.body != f.Body || t.kind == PrismFaceUnknown || len(f.Origins) != 1 || len(f.Upstream) != 0 {
		return false
	}
	switch f.Surface.(type) {
	case surfacegeom.Plane, surfacegeom.Cylinder:
		if t.kind == PrismFaceWall && f.CapRecord == nil {
			t.sealed = true
		}
		if t.kind != PrismFaceWall && f.CapRecord != nil && f.CapRecord.ZDenoted != nil && f.CapRecord.ZDenoted.Num().BitLen() <= 4096 && f.CapRecord.ZDenoted.Denom().BitLen() <= 4096 {
			t.sealed = true
			t.cap = f.CapRecord
			t.capValue = *f.CapRecord
			t.capLevel = new(big.Rat).Set(f.CapRecord.ZDenoted)
		}
	}
	if !t.sealed {
		return false
	}
	t.topology, t.sealed = capturePrismFaceTopology(f)
	if !t.sealed {
		return false
	}
	t.origin = f.Origins[0]
	t.surface = f.Surface
	t.centroid = f.Centroid
	t.scalars = prismFaceScalars(f)
	t.flags = prismFaceFlags(f)
	f.prismFaceToken = t
	return true
}

func prismFaceScalars(f *Face) [6]uint64 {
	return [6]uint64{math.Float64bits(f.Area), math.Float64bits(f.AreaBound), math.Float64bits(f.AxialDelta), math.Float64bits(f.NormalBound), math.Float64bits(f.HeldNormalBound), math.Float64bits(f.TagDeparture)}
}
func prismFaceFlags(f *Face) [5]bool {
	return [5]bool{f.HasCentroid, f.HasAxialDelta, f.HeldPlanar, f.Reversed, f.RetainedTag}
}

// PrismConstruction returns a token only for the exact face and body stamped at Build.
func (f *Face) PrismConstruction() (PrismFaceToken, bool) {
	if f == nil {
		return PrismFaceToken{}, false
	}
	t := f.prismFaceToken
	if t.kind == PrismFaceUnknown || t.face != f || t.body == nil || t.body.prismConstructionState != 2 || f.Body != t.body || t.body.Origin.Producer != t.producer {
		return PrismFaceToken{}, false
	}
	return t, true
}

// Matches compares an actual built face with its recorded construction identity.
func (t PrismFaceToken) Matches(face *Face, body *Body, producer provenance.Producer, kind PrismFaceKind, loop, segment int) bool {
	return t.kind != PrismFaceUnknown && t.face == face && t.body == body && t.producer == producer && t.kind == kind && t.loop == loop && t.segment == segment && face != nil && face.Body == body && body != nil && body.prismConstructionState == 2 && body.Origin.Producer == producer
}

// MatchesSealed reports whether the restricted construction snapshot still
// matches. The caller charges bounded rational comparison before calling it.
func (t PrismFaceToken) MatchesSealed(face *Face) bool {
	if !t.sealed || face == nil || prismFaceScalars(face) != t.scalars || prismFaceFlags(face) != t.flags || !samePrismCentroid(face.Centroid, t.centroid) || len(face.Origins) != 1 || len(face.Upstream) != 0 || face.Origins[0] != t.origin {
		return false
	}
	switch got := face.Surface.(type) {
	case surfacegeom.Plane:
		want, ok := t.surface.(surfacegeom.Plane)
		if !ok || !samePrismFrame(got.Frame, want.Frame) {
			return false
		}
	case surfacegeom.Cylinder:
		want, ok := t.surface.(surfacegeom.Cylinder)
		if !ok || !samePrismVec(got.Origin, want.Origin) || !samePrismVec(got.Axis, want.Axis) || got.Radius != want.Radius || math.Float64bits(got.Radius.Base()) != math.Float64bits(want.Radius.Base()) {
			return false
		}
	default:
		return false
	}
	if t.cap == nil {
		return face.CapRecord == nil
	}
	c := face.CapRecord
	if c != t.cap || c == nil || !samePrismFrame(c.Frame, t.capValue.Frame) || !samePrismTransform(c.Xform, t.capValue.Xform) || math.Float64bits(c.Z) != math.Float64bits(t.capValue.Z) || c.ZDenoted == nil || c.ZDenoted != t.capValue.ZDenoted || t.capLevel == nil {
		return false
	}
	return c.ZDenoted.Num().Cmp(t.capLevel.Num()) == 0 && c.ZDenoted.Denom().Cmp(t.capLevel.Denom()) == 0
}

func samePrismVec(a, b r3.Vec) bool {
	return math.Float64bits(a.X) == math.Float64bits(b.X) && math.Float64bits(a.Y) == math.Float64bits(b.Y) && math.Float64bits(a.Z) == math.Float64bits(b.Z)
}
func samePrismFrame(a, b r3.Frame) bool {
	return samePrismVec(a.Origin(), b.Origin()) && samePrismVec(a.U(), b.U()) && samePrismVec(a.V(), b.V()) && samePrismVec(a.N(), b.N())
}
func samePrismTransform(a, b r3.Transform) bool {
	x, y := a.Basis(), b.Basis()
	return samePrismVec(x.EX, y.EX) && samePrismVec(x.EY, y.EY) && samePrismVec(x.EZ, y.EZ) && samePrismVec(a.Translation(), b.Translation())
}
func samePrismCentroid(a, b measurement.VecMeasurement) bool {
	return a == b && samePrismVec(a.Value, b.Value) && math.Float64bits(a.Bound.Base()) == math.Float64bits(b.Bound.Base())
}
