package export

import (
	"bytes"
	"context"
	"fmt"
	"io"
	"math"
	"strconv"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// igesEntity is one entity of the file: its type and form, whether it is physically dependent on another
// (every entity below the solid), and its parameters after the type, as text.
type igesEntity struct {
	typ, form int
	dependent bool
	params    []string
}

// igesFile collects the entities in directory order. An entity's directory sequence number, the number the others
// point at it with, is odd: its two directory records start at 2·index+1.
type igesFile struct{ entities []igesEntity }

func (f *igesFile) add(typ, form int, dependent bool, params ...string) int {
	f.entities = append(f.entities, igesEntity{typ: typ, form: form, dependent: dependent, params: params})
	return 2*len(f.entities) - 1
}

// igesReal formats a float as an IGES real: the shortest text that reads back as the same float, always with a decimal
// point and an upper case E exponent.
func igesReal(x float64) string {
	s := strconv.FormatFloat(x, 'g', -1, 64)
	mantissa, exponent, hasExponent := strings.Cut(s, "e")
	if !strings.Contains(mantissa, ".") {
		mantissa += ".0"
	}
	if hasExponent {
		return mantissa + "E" + exponent
	}
	return mantissa
}

func igesInt(n int) string { return strconv.Itoa(n) }

// IGES entity type numbers written here.
const (
	igesLine       = 110
	igesPlane      = 108
	igesVertexList = 502
	igesEdgeList   = 504
	igesLoop       = 508
	igesFace       = 510
	igesShell      = 514
	igesSolid      = 186
)

// IGES writes body as an IGES 5.3 file of manifold solid B-reps (agent-docs/iges-export-design.md): the verified
// tessellation at tol, one planar face per triangle, so any solid the evaluator can tessellate is written as the
// planar solid ReadIGES reads, and a read back mesh has the body's own triangles. A body of several lumps writes
// one solid per lump and a cavity a void shell of its solid. The file is millimetres with a model space scale of 1.
//
// ctx, w and body MUST NOT be nil, each otherwise decad.ErrDegenerate. A body that is not a valid solid is
// decad.ErrNotSolid, a mesh whose boundary is not verified, whose edges are not each used twice or that does not map
// to the body's shells is decad.ErrUnsupported, and a failure writes nothing.
func IGES(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value) error {
	if ctx == nil {
		return fmt.Errorf("export: IGES: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: IGES: %w: nil writer", decad.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if body == nil {
		return fmt.Errorf("export: IGES: %w: nil body", decad.ErrDegenerate)
	}
	if body.Kind() != decad.BodySolid || !body.IsSolid() {
		return fmt.Errorf("export: IGES: %w: body does not enclose a valid region", decad.ErrNotSolid)
	}
	mesh, err := body.Tessellate(ctx, tol, decad.WithVerification(decad.VerifyBoundary))
	if err != nil {
		return err
	}
	if !mesh.BoundaryVerified() {
		return fmt.Errorf("export: IGES: %w: mesh boundary is not verified", decad.ErrUnsupported)
	}
	vertices, triangles := mesh.Vertices(), mesh.Triangles()
	components, count, err := checkMesh(ctx, vertices, triangles)
	if err != nil {
		return reprefix(err, "IGES")
	}
	if shells := body.Shells(); count != len(shells) {
		return fmt.Errorf("export: IGES: %w: the mesh holds %d connected shells and the body %d",
			decad.ErrUnsupported, count, len(shells))
	}
	solids, err := meshSolids(body, mesh.SourceFaces(), vertices, triangles, components, count)
	if err != nil {
		return reprefix(err, "IGES")
	}
	file, err := assembleIGES(ctx, vertices, triangles, components, count, solids)
	if err != nil {
		return err
	}
	out := file.encode()
	if n, err := w.Write(out); err != nil {
		return fmt.Errorf("export: IGES write failed: %w", err)
	} else if n != len(out) {
		return fmt.Errorf("export: IGES write failed: %w", io.ErrShortWrite)
	}
	return nil
}

// reprefix names format in the message of an error checkMesh or meshSolids worded for STEP, and keeps the error
// itself wrapped.
func reprefix(err error, format string) error {
	return renamedError{msg: strings.Replace(err.Error(), "export: STEP:", "export: "+format+":", 1), err: err}
}

type renamedError struct {
	msg string
	err error
}

func (e renamedError) Error() string { return e.msg }
func (e renamedError) Unwrap() error { return e.err }

// assembleIGES builds the entities of the solids: one vertex list for every vertex, one edge list for every mesh edge
// of straight lines, and for each triangle a plane, a loop of its three edges and a face, then a shell of the faces
// of each connected facet set and the solid of each lump.
func assembleIGES(ctx context.Context, vertices []r3.Vec, triangles [][3]int, components []int, count int,
	solids []meshSolid) (*igesFile, error) {
	f := &igesFile{}
	// The vertex list, then a line for every distinct edge, from its lower vertex to its higher, in the order the
	// triangles first use them.
	vlist := []string{igesInt(len(vertices))}
	for _, v := range vertices {
		vlist = append(vlist, igesReal(v.X), igesReal(v.Y), igesReal(v.Z))
	}
	vertexList := f.add(igesVertexList, 1, true, vlist...)
	edgeIndex := map[edgeKey]int{}
	var edges []edgeKey
	for _, tri := range triangles {
		for j := range 3 {
			key := orderedEdge(tri[j], tri[(j+1)%3])
			if _, ok := edgeIndex[key]; !ok {
				edgeIndex[key] = len(edges) + 1
				edges = append(edges, key)
			}
		}
	}
	elist := []string{igesInt(len(edges))}
	for _, e := range edges {
		a, b := vertices[e.a], vertices[e.b]
		line := f.add(igesLine, 0, true,
			igesReal(a.X), igesReal(a.Y), igesReal(a.Z), igesReal(b.X), igesReal(b.Y), igesReal(b.Z))
		elist = append(elist, igesInt(line), igesInt(vertexList), igesInt(e.a+1), igesInt(vertexList), igesInt(e.b+1))
	}
	edgeList := f.add(igesEdgeList, 1, true, elist...)

	planes := map[[4]float64]int{}
	faces := make([]int, len(triangles))
	for t, tri := range triangles {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		normal := b.Sub(a).Cross(c.Sub(a))
		length := normal.Len()
		if !(length > 0) || math.IsInf(length, 0) {
			return nil, fmt.Errorf("export: IGES: %w: facet %d has no area", decad.ErrUnsupported, t)
		}
		normal = normal.Scale(1 / length)
		key := [4]float64{normal.X, normal.Y, normal.Z, normal.Dot(a)}
		plane, ok := planes[key]
		if !ok {
			plane = f.add(igesPlane, 0, true, igesReal(key[0]), igesReal(key[1]), igesReal(key[2]), igesReal(key[3]),
				"0", "0.0", "0.0", "0.0", "0.0")
			planes[key] = plane
		}
		loop := []string{"3"}
		for j := range 3 {
			u, v := tri[j], tri[(j+1)%3]
			key := orderedEdge(u, v)
			forward := "0"
			if u == key.a {
				forward = "1"
			}
			loop = append(loop, "0", igesInt(edgeList), igesInt(edgeIndex[key]), forward, "0")
		}
		loopDE := f.add(igesLoop, 1, true, loop...)
		faces[t] = f.add(igesFace, 1, true, igesInt(plane), "1", "1", igesInt(loopDE))
	}
	shells := make([]int, count)
	for c := range count {
		var members []string
		n := 0
		for t, face := range faces {
			if components[t] == c {
				members = append(members, igesInt(face), "1")
				n++
			}
		}
		shells[c] = f.add(igesShell, 1, true, append([]string{igesInt(n)}, members...)...)
	}
	for _, s := range solids {
		params := []string{igesInt(shells[s.outer]), "1", igesInt(len(s.voids))}
		for _, v := range s.voids {
			params = append(params, igesInt(shells[v]), "1")
		}
		f.add(igesSolid, 0, false, params...)
	}
	return f, nil
}

// igesGlobal is the global section's text: the delimiters, the identifiers, the number formats, a model space scale
// of 1 in millimetres, a resolution of a micron and IGES 5.3.
const igesGlobal = "1H,,1H;,5Hdecad,8Hbody.igs,5Hdecad,3H1.0,32,38,6,308,15,5Hdecad,1.0,2,2HMM,1,0.01,,1.0E-06,,,,11,0,;"

// encode writes the file's five sections: the start, global and directory entry and parameter data records of 80
// columns, and the terminate record.
func (f *igesFile) encode() []byte {
	var out bytes.Buffer
	record := func(text string, section byte, seq int) {
		fmt.Fprintf(&out, "%-72s%c%7d\n", text, section, seq)
	}
	record("decad IGES export: the planar faces of a verified tessellation", 'S', 1)
	global := wrapFields(igesGlobal, 72)
	for i, line := range global {
		record(line, 'G', i+1)
	}
	// Parameter data first, to learn each entity's records.
	type placed struct {
		start int
		lines []string
	}
	pieces := make([]placed, len(f.entities))
	next := 1
	for i, e := range f.entities {
		fields := append([]string{igesInt(e.typ)}, e.params...)
		lines := wrapFields(strings.Join(fields, ",")+";", 64)
		pieces[i] = placed{start: next, lines: lines}
		next += len(lines)
	}
	for i, e := range f.entities {
		status := "00000000"
		if e.dependent {
			status = "00010000"
		}
		seq := 2*i + 1
		fmt.Fprintf(&out, "%8d%8d%8d%8d%8d%8d%8d%8d%s%c%7d\n", e.typ, pieces[i].start, 0, 0, 0, 0, 0, 0, status, 'D', seq)
		fmt.Fprintf(&out, "%8d%8d%8d%8d%8d%8s%8s%8s%8s%c%7d\n", e.typ, 0, 0, len(pieces[i].lines), e.form, "", "", "", "", 'D', seq+1)
	}
	for i, p := range pieces {
		for k, line := range p.lines {
			fmt.Fprintf(&out, "%-64s%8d%c%7d\n", line, 2*i+1, 'P', p.start+k)
		}
	}
	fmt.Fprintf(&out, "S%7dG%7dD%7dP%7d%sT%7d\n", 1, len(global), 2*len(f.entities), next-1, strings.Repeat(" ", 40), 1)
	return out.Bytes()
}

// wrapFields cuts a comma and semicolon delimited record into lines of at most width columns, each ending after a
// delimiter, so that no parameter is split between two lines.
func wrapFields(text string, width int) []string {
	var lines []string
	for len(text) > width {
		cut := strings.LastIndexAny(text[:width], ",;")
		if cut < 0 {
			cut = width - 1 // no delimiter to cut at: a parameter longer than a line, which no entity here has
		}
		lines = append(lines, text[:cut+1])
		text = text[cut+1:]
	}
	return append(lines, text)
}
