package decad

import (
	"fmt"
	"go/types"
	"maps"
	"reflect"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/stretchr/testify/require"
)

// digestGraph walks the static type graph the digest follows from Body.
type digestGraph struct {
	ms       *moduleSource
	named    []*types.Named // every non-interface named type of the checked packages
	seen     map[types.Type]struct{}
	findings map[string]struct{}
	hits     map[string]struct{}
	anyField map[string]struct{}
}

func newDigestGraph(ms *moduleSource) *digestGraph {
	g := &digestGraph{ms: ms, seen: map[types.Type]struct{}{}, findings: map[string]struct{}{}, hits: map[string]struct{}{}, anyField: map[string]struct{}{}}
	for _, tp := range ms.pkgs {
		scope := tp.pkg.Scope()
		for _, name := range scope.Names() {
			tn, ok := scope.Lookup(name).(*types.TypeName)
			if !ok || tn.IsAlias() {
				continue
			}
			n, ok := tn.Type().(*types.Named)
			if !ok || n.TypeParams().Len() > 0 {
				continue
			}
			if _, isIface := n.Underlying().(*types.Interface); isIface {
				continue
			}
			g.named = append(g.named, n)
		}
	}
	return g
}

// typeKey names a type by package path and name, the same way for reflect and go/types.
func typeKey(pkgPath, name string) string { return pkgPath + "." + name }

func namedKey(n *types.Named) string {
	if n.Obj().Pkg() == nil {
		return n.Obj().Name()
	}
	return typeKey(n.Obj().Pkg().Path(), n.Obj().Name())
}

// classified reports whether field of owner is one the digest skips or refuses, and records the hit.
func (g *digestGraph) classified(owner *types.Named, field string) bool {
	if owner == nil {
		return false
	}
	key := namedKey(owner)
	for t, names := range digestSkipFields {
		if typeKey(t.PkgPath(), t.Name()) != key || !slices.Contains(names, field) {
			continue
		}
		g.hits["skip "+key+"."+field] = struct{}{}
		return true
	}
	for t, fields := range digestRefuseFields {
		if _, ok := fields[field]; ok && typeKey(t.PkgPath(), t.Name()) == key {
			g.hits["refuse "+key+"."+field] = struct{}{}
			return true
		}
	}
	return false
}

func (g *digestGraph) visit(t types.Type, path string, owner *types.Named) {
	switch t := t.(type) {
	case *types.Named:
		g.visitNamed(t, path)
	case *types.Alias:
		g.visit(types.Unalias(t), path, owner)
	case *types.Pointer:
		g.visit(t.Elem(), path, nil)
	case *types.Slice:
		g.visit(t.Elem(), path, nil)
	case *types.Array:
		g.visit(t.Elem(), path, nil)
	case *types.Map:
		g.visit(t.Key(), path, nil)
		g.visit(t.Elem(), path, nil)
	case *types.Struct:
		g.visitStruct(t, path, owner)
	case *types.Signature, *types.Chan:
		g.findings[fmt.Sprintf("%s: %s", path, t)] = struct{}{}
	case *types.Basic:
		if t.Kind() == types.UnsafePointer {
			g.findings[path+": unsafe.Pointer"] = struct{}{}
		}
	case *types.Interface:
		g.visitInterface(t, path)
	}
}

func (g *digestGraph) visitNamed(n *types.Named, path string) {
	if _, ok := g.seen[n]; ok {
		return
	}
	g.seen[n] = struct{}{}
	obj := n.Obj()
	if obj.Pkg() != nil {
		switch obj.Pkg().Path() {
		case "sync", "sync/atomic":
			g.findings[fmt.Sprintf("%s: %s.%s", path, obj.Pkg().Path(), obj.Name())] = struct{}{}
			return
		}
		full := obj.Pkg().Path() + "." + obj.Name()
		if full == g.ms.path+"/internal/topo.Document" || full == g.ms.path+".Document" {
			return
		}
		for t := range digestRefuseTypes {
			if typeKey(t.PkgPath(), t.Name()) == full {
				g.hits["refuse "+full] = struct{}{}
				return
			}
		}
	}
	if n.TypeParams().Len() > 0 {
		g.visit(n.Underlying(), path+"/"+obj.Name(), n)
		return
	}
	if st, ok := n.Underlying().(*types.Struct); ok {
		g.visitStruct(st, path+"/"+obj.Name(), n)
		return
	}
	g.visit(n.Underlying(), path+"/"+obj.Name(), nil)
}

func (g *digestGraph) visitStruct(st *types.Struct, path string, owner *types.Named) {
	for f := range st.Fields() {
		if g.classified(owner, f.Name()) {
			continue
		}
		if f.Name() == "Owner" {
			if _, isIface := f.Type().Underlying().(*types.Interface); isIface {
				continue
			}
		}
		g.visit(f.Type(), path+"."+f.Name(), nil)
	}
}

func (g *digestGraph) visitInterface(it *types.Interface, path string) {
	if it.NumMethods() == 0 && it.NumEmbeddeds() == 0 {
		g.anyField[path] = struct{}{}
		return
	}
	for _, n := range g.named {
		if types.Implements(n, it) || types.Implements(types.NewPointer(n), it) {
			g.visit(n, path+"<"+n.Obj().Name()+">", nil)
		}
	}
}

// TestDigestTypeCoverage requires every value the digest can reach from a Body to be encodable or classified. It
// walks the static type graph of Body, following each non-empty interface into every checked type that implements it
// (topo.Payload among them), and fails on a synchronization value, function, channel or unsafe pointer that
// digestSkipFields does not skip and digestRefuseTypes or digestRefuseFields does not refuse. A new cache or lazy
// field added to a reachable type fails here until someone classifies it. Condition C1 of
// agent-docs/body-digest-design.md.
func TestDigestTypeCoverage(t *testing.T) {
	ms := loadModuleDecls(t)
	g := newDigestGraph(ms)
	root := ms.pkgs[ms.path].pkg.Scope().Lookup("Body").Type()
	g.visit(root, "Body", nil)

	require.Empty(t, slices.Sorted(maps.Keys(g.findings)), "unclassified unencodable values reachable from Body")

	// An interface{} field has no static implementers. Its dynamic values are checked when the digest encodes them,
	// where an unencodable one is ErrUnsupported. A new such field needs a decision here.
	require.Equal(t, []string{
		"Body/Body.n/Body.PrivateShift",
	}, slices.Sorted(maps.Keys(g.anyField)))

	// Every classification must name a field or type that is reachable, so a rename cannot leave a stale entry.
	want := []string{

		"refuse github.com/lestrrat-3d/decad/internal/occupancy.Leaf.inner",
		"refuse github.com/lestrrat-3d/decad/internal/occupancy.Leaf.pieces",
		"refuse github.com/lestrrat-3d/sketch.FitSpline",
		"refuse github.com/lestrrat-3d/sketch.Sketch",
		"skip github.com/lestrrat-3d/decad.Body.digest",
		"skip github.com/lestrrat-3d/decad.Body.pairReports",
		"skip github.com/lestrrat-3d/decad.Operation.step",
		"skip github.com/lestrrat-3d/decad.blindGroupProof.cache",
		"skip github.com/lestrrat-3d/decad/internal/occupancy.Record.grid",
		"skip github.com/lestrrat-3d/decad/internal/topo.Body.PlanarConvexity",
		"skip github.com/lestrrat-3d/decad/internal/topo.Body.PlanarHints",
		"skip github.com/lestrrat-3d/decad/internal/topo.Body.PlanarSnapshot",
		"skip github.com/lestrrat-3d/decad/internal/topo.Body.SweepRadii",
		"skip github.com/lestrrat-3d/decad/internal/topo.Body.Tess",
	}
	require.Equal(t, want, slices.Sorted(maps.Keys(g.hits)))
}

// TestDigestSideTables requires every field of Document to be classified as one the digest reads or one it leaves
// out with a reason, so a table added later that the evaluator reads cannot go unnoticed. A map keyed by *Body is
// the shape that matters: the digest reads such a table through bodyDigestSideTables. Condition C1.
func TestDigestSideTables(t *testing.T) {
	read := "read by the digest"
	classes := map[string]string{
		"n":                  "topo.Document: the live set and counters; the settings the evaluator reads are encoded by documentSettingsKey and the counters only name the uncommitted result",
		"gapReach":           read + " (documentSettingsKey)",
		"gapErr":             read + " (documentSettingsKey)",
		"chainTol":           read + " (documentSettingsKey)",
		"chainErr":           read + " (documentSettingsKey)",
		"rimSplit":           read + " (documentSettingsKey)",
		"params":             "the parameter table feeds feature builds before a body exists; no evaluation reads it",
		"ledger":             "records which parameters a build read; no evaluation reads it",
		"groups":             "containers; an evaluation reads no container",
		"operations":         "the history in order; an evaluation reads an operation through producers",
		"producers":          read + " (bodyDigestSideTables)",
		"consumers":          "only the paired and blind Cut routes read it, and Intersect runs neither",
		"circleBuilds":       read + " (bodyDigestSideTables)",
		"circlePlacements":   read + " (bodyDigestSideTables)",
		"identityPlacements": read + " (bodyDigestSideTables)",
	}
	bodyKeyed := map[string]bool{}
	dt := reflect.TypeFor[Document]()
	for f := range dt.Fields() {
		_, ok := classes[f.Name]
		require.True(t, ok, "Document.%s is not classified for the digest: read it in bodyDigestSideTables or give a reason", f.Name)
		if f.Type.Kind() == reflect.Map && f.Type.Key() == bodyPtrType {
			bodyKeyed[f.Name] = true
		}
	}
	require.Len(t, classes, dt.NumField(), "a classified field no longer exists")
	require.Equal(t, map[string]bool{
		"producers": true, "consumers": true, "circleBuilds": true, "circlePlacements": true, "identityPlacements": true,
	}, bodyKeyed)

	// The hook returns exactly the tables classified as read, in order.
	doc := New()
	body := bornBody(&Body{})
	doc.n.Owner = doc
	body.n.Doc = &doc.n
	op := &Operation{}
	cb := &prismrecord.CircleBuildWitness{}
	cp := &circlePlacementWitness{}
	doc.producers[body] = op
	doc.circleBuilds[body] = cb
	doc.circlePlacements[body] = cp
	doc.consumers[body] = []*Operation{op}
	doc.identityPlacements = map[*Body]struct{}{body: {}}
	got := bodyDigestSideTables(reflect.ValueOf(body))
	require.Len(t, got, 4)
	require.Same(t, op, got[0].Interface())
	require.Same(t, cb, got[1].Interface())
	require.Same(t, cp, got[2].Interface())
	require.True(t, got[3].Bool())
	require.Nil(t, bodyDigestSideTables(reflect.ValueOf(&op)))
}
