package decad

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file joins the root topology handles to their internal/topo nodes
// (docs/evaluator-design.md §3): handleAlloc gives a builder nodes that are
// born inside their handles, and the xxxOf helpers map a node back to its
// handle through the node's Owner.
//
// PHASE H SLICE (HS) SCAFFOLDING: everything below the "throwaway" marker
// exists only on the probe branch, where link and twin fields are held twice.

// handleAlloc is root's topo.Alloc: each node is the n field of a fresh handle,
// with Owner pointing back at that handle.
type handleAlloc struct{}

func (handleAlloc) NewBody() *topo.Body     { h := new(Body); h.n.Owner = h; return &h.n }
func (handleAlloc) NewLump() *topo.Lump     { h := new(Lump); h.n.Owner = h; return &h.n }
func (handleAlloc) NewShell() *topo.Shell   { h := new(Shell); h.n.Owner = h; return &h.n }
func (handleAlloc) NewFace() *topo.Face     { h := new(Face); h.n.Owner = h; return &h.n }
func (handleAlloc) NewLoop() *topo.Loop     { h := new(Loop); h.n.Owner = h; return &h.n }
func (handleAlloc) NewEdge() *topo.Edge     { h := new(Edge); h.n.Owner = h; return &h.n }
func (handleAlloc) NewVertex() *topo.Vertex { h := new(Vertex); h.n.Owner = h; return &h.n }
func (d *Document) env(ref producerID) topo.Env {
	return topo.Env{Doc: &d.n, Ref: provenance.Producer(ref), Alloc: handleAlloc{}}
}

// handleOf maps a node's Owner to its handle. A node no root allocator built
// has no Owner of the right type: that is an evaluator invariant failure.
func handleOf[H any](owner any) *H {
	h, ok := owner.(*H)
	if !ok {
		panic(fmt.Sprintf("decad: topology node owned by %T, not %T", owner, h))
	}
	return h
}

func vertexOf(n *topo.Vertex) *Vertex {
	if n == nil {
		return nil
	}
	return handleOf[Vertex](n.Owner)
}

func edgeOf(n *topo.Edge) *Edge {
	if n == nil {
		return nil
	}
	return handleOf[Edge](n.Owner)
}

func loopOf(n *topo.Loop) *Loop {
	if n == nil {
		return nil
	}
	return handleOf[Loop](n.Owner)
}

func faceOf(n *topo.Face) *Face {
	if n == nil {
		return nil
	}
	return handleOf[Face](n.Owner)
}

func shellOf(n *topo.Shell) *Shell {
	if n == nil {
		return nil
	}
	return handleOf[Shell](n.Owner)
}

func lumpOf(n *topo.Lump) *Lump {
	if n == nil {
		return nil
	}
	return handleOf[Lump](n.Owner)
}

func bodyOf(n *topo.Body) *Body {
	if n == nil {
		return nil
	}
	return handleOf[Body](n.Owner)
}

func documentOf(n *topo.Document) *Document {
	if n == nil {
		return nil
	}
	return handleOf[Document](n.Owner)
}

// mapNodes maps a node slice to its handles, nil for an empty input, matching
// the accessors' append([]*T(nil), ...) copies.
func mapNodes[N any, H any](ns []*N, of func(*N) *H) []*H {
	if len(ns) == 0 {
		return nil
	}
	out := make([]*H, len(ns))
	for i, n := range ns {
		out[i] = of(n)
	}
	return out
}

func coEdgesOf(ns []topo.CoEdge) []CoEdge {
	if len(ns) == 0 {
		return nil
	}
	out := make([]CoEdge, len(ns))
	for i, c := range ns {
		out[i] = CoEdge{n: c, edge: edgeOf(c.Edge), forward: c.Forward}
	}
	return out
}

func featureRefOf(r provenance.Ref) FeatureRef {
	return FeatureRef{producer: producerID(r.Producer), Role: r.Role}
}

func featureRefsOf(rs []provenance.Ref) []FeatureRef {
	if len(rs) == 0 {
		return nil
	}
	out := make([]FeatureRef, len(rs))
	for i, r := range rs {
		out[i] = featureRefOf(r)
	}
	return out
}

// ---- throwaway (HS only) ----

// publishNodeBody fills every legacy link and twin field of a node-built body's
// handles from their nodes, so root code that still reads the legacy fields
// sees the same topology. d is the document the body was built for.
func publishNodeBody(d *Document, nb *topo.Body) *Body {
	h := bodyOf(nb)
	h.doc = d
	h.origin = featureRefOf(nb.Origin)
	h.lumps = mapNodes(nb.Lumps, lumpOf)
	h.volume = measurementFromInternal(nb.Volume)
	h.area = measurementFromInternal(nb.Area)
	h.centroid = vecMeasurementFromInternal(nb.Centroid)
	h.bounds = boxFromInternal(nb.Bounds)
	h.kind = BodyKind(nb.Kind)
	rec, ok := nb.Payload.(prismrecord.Payload)
	if !ok {
		panic(fmt.Sprintf("decad: HS publishes prism bodies only, not %T", nb.Payload))
	}
	h.payload = prismPayload{Payload: rec}
	for _, ln := range nb.Lumps {
		lumpOf(ln).shells = mapNodes(ln.Shells, shellOf)
		for _, sn := range ln.Shells {
			shellOf(sn).faces = mapNodes(sn.Faces, faceOf)
		}
	}
	for _, fn := range nb.Faces() {
		f := faceOf(fn)
		f.surface = publicSurface(fn.Surface)
		f.loops = mapNodes(fn.Loops, loopOf)
		f.origins = featureRefsOf(fn.Origins)
		f.body = bodyOf(fn.Body)
		f.centroid = vecMeasurementFromInternal(fn.Centroid)
		for _, ln := range fn.Loops {
			loopOf(ln).coedges = coEdgesOf(ln.CoEdges)
		}
	}
	for _, en := range nb.Edges() {
		e := edgeOf(en)
		e.curve = publicCurve(en.Curve)
		e.start = vertexOf(en.Start)
		e.end = vertexOf(en.End)
		e.faces = mapNodes(en.Faces, faceOf)
	}
	return h
}

// dualCheck reports the first legacy link or twin field of b's topology that
// differs from its node field mapped through Owner. A body whose node has no
// Owner was built by a root literal and has nothing to compare.
func dualCheck(b *Body) error {
	if b.n.Owner == nil {
		return nil
	}
	nb := &b.n
	switch {
	case bodyOf(nb) != b:
		return fmt.Errorf("body owner")
	case b.doc != documentOf(nb.Doc):
		return fmt.Errorf("body doc")
	case b.origin != featureRefOf(nb.Origin):
		return fmt.Errorf("body origin")
	case !slices.Equal(b.lumps, mapNodes(nb.Lumps, lumpOf)):
		return fmt.Errorf("body lumps")
	case b.volume != measurementFromInternal(nb.Volume), b.area != measurementFromInternal(nb.Area):
		return fmt.Errorf("body measurements")
	case b.centroid != vecMeasurementFromInternal(nb.Centroid), b.bounds != boxFromInternal(nb.Bounds):
		return fmt.Errorf("body centroid or bounds")
	case b.kind != BodyKind(nb.Kind):
		return fmt.Errorf("body kind")
	}
	for _, ln := range nb.Lumps {
		if !slices.Equal(lumpOf(ln).shells, mapNodes(ln.Shells, shellOf)) {
			return fmt.Errorf("lump shells")
		}
		for _, sn := range ln.Shells {
			if !slices.Equal(shellOf(sn).faces, mapNodes(sn.Faces, faceOf)) {
				return fmt.Errorf("shell faces")
			}
		}
	}
	for _, fn := range nb.Faces() {
		f := faceOf(fn)
		switch {
		case f.surface != publicSurface(fn.Surface):
			return fmt.Errorf("face surface")
		case !slices.Equal(f.loops, mapNodes(fn.Loops, loopOf)):
			return fmt.Errorf("face loops")
		case !slices.Equal(f.origins, featureRefsOf(fn.Origins)):
			return fmt.Errorf("face origins")
		case f.body != bodyOf(fn.Body), f.body != b:
			return fmt.Errorf("face body")
		case f.centroid != vecMeasurementFromInternal(fn.Centroid):
			return fmt.Errorf("face centroid")
		}
		for _, ln := range fn.Loops {
			if !slices.Equal(loopOf(ln).coedges, coEdgesOf(ln.CoEdges)) {
				return fmt.Errorf("loop coedges")
			}
		}
	}
	for _, en := range nb.Edges() {
		e := edgeOf(en)
		switch {
		case e.curve != publicCurve(en.Curve):
			return fmt.Errorf("edge curve")
		case e.start != vertexOf(en.Start), e.end != vertexOf(en.End):
			return fmt.Errorf("edge ends")
		case !slices.Equal(e.faces, mapNodes(en.Faces, faceOf)):
			return fmt.Errorf("edge faces")
		}
	}
	return nil
}

// hsDualCheck panics when a committed body's legacy fields disagree with its
// nodes.
func hsDualCheck(b *Body) {
	if err := dualCheck(b); err != nil {
		panic("decad: HS dual fields disagree: " + err.Error())
	}
}
