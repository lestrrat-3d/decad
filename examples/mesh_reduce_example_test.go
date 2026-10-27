package examples_test

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// ReduceMesh thins a dense mesh by clustering its vertices into cubic cells: the
// vertices in a cell become one at their mean. Use it to shrink a scan or a fine
// tessellation before checking or exporting it. The report gives the greatest
// distance a vertex moved. Here a 4 by 4 grid of unit squares is thinned with
// 2 mm cells.
func Example_mesh_reduce() {
	var src strings.Builder
	for y := range 5 {
		for x := range 5 {
			fmt.Fprintf(&src, "v %d %d 0\n", x, y)
		}
	}
	at := func(x, y int) int { return y*5 + x + 1 }
	for y := range 4 {
		for x := range 4 {
			fmt.Fprintf(&src, "f %d %d %d\nf %d %d %d\n",
				at(x, y), at(x+1, y), at(x+1, y+1), at(x, y), at(x+1, y+1), at(x, y+1))
		}
	}
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(src.String()))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	reduced, report, err := decad.ReduceMesh(context.Background(), mesh, units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to reduce: %s\n", err)
		return
	}
	fmt.Printf("triangles: %d -> %d\n", mesh.Report().Triangles, reduced.Report().Triangles)
	fmt.Printf("vertices moved at most %.2f mm\n", report.MaxVertexMove.Base())
	// Output:
	// triangles: 32 -> 8
	// vertices moved at most 0.71 mm
}

// ReduceMeshWithin thins a mesh as ReduceMesh does, but you state how far the
// result may lie from the original surface and it proves the distance: the report's
// SurfaceBound holds from the original to the result and back. Here a 8 by 8 grid
// of unit squares is thinned within 0.8 mm.
func Example_mesh_reduce_within() {
	var src strings.Builder
	for y := range 9 {
		for x := range 9 {
			fmt.Fprintf(&src, "v %d %d 0\n", x, y)
		}
	}
	at := func(x, y int) int { return y*9 + x + 1 }
	for y := range 8 {
		for x := range 8 {
			fmt.Fprintf(&src, "f %d %d %d\nf %d %d %d\n",
				at(x, y), at(x+1, y), at(x+1, y+1), at(x, y), at(x+1, y+1), at(x, y+1))
		}
	}
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(src.String()))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	reduced, report, err := decad.ReduceMeshWithin(context.Background(), mesh, units.Millimeters(0.8))
	if err != nil {
		fmt.Printf("failed to reduce: %s\n", err)
		return
	}
	fmt.Printf("thinned: %t, within 0.8 mm: %t\n",
		reduced.Report().Triangles < mesh.Report().Triangles, report.SurfaceBound.Base() <= 0.8)
	// Output:
	// thinned: true, within 0.8 mm: true
}

// CollapseMesh thins a closed mesh by edge collapse until it has at most the triangles you
// ask for: it merges the shortest edges first, each at its midpoint, and keeps the surface
// closed. Here a 12-sided bipyramid of 24 triangles is thinned to 12.
func Example_mesh_collapse() {
	var src strings.Builder
	src.WriteString("v 0 0 3\nv 0 0 -3\n")
	for i := range 12 {
		a := 2 * math.Pi * float64(i) / 12
		fmt.Fprintf(&src, "v %.6f %.6f 0\n", 5*math.Cos(a), 5*math.Sin(a))
	}
	for i := range 12 {
		j := (i + 1) % 12
		fmt.Fprintf(&src, "f 1 %d %d\nf 2 %d %d\n", 3+i, 3+j, 3+j, 3+i)
	}
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(src.String()))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	thin, report, err := decad.CollapseMesh(context.Background(), mesh, 12)
	if err != nil {
		fmt.Printf("failed to collapse: %s\n", err)
		return
	}
	fmt.Printf("triangles: %d -> %d, closed: %t\n", mesh.Report().Triangles, thin.Report().Triangles, thin.Report().Closed)
	fmt.Printf("edges merged: %d\n", report.MergedVertices)
	// Output:
	// triangles: 24 -> 12, closed: true
	// edges merged: 6
}

// RemeshMesh rebuilds a closed mesh from near-equilateral triangles of about the edge length
// you ask for: it splits long edges, merges short ones, flips edges toward six neighbours to a
// vertex and smooths. Here a 12-sided bipyramid, whose triangles are long slivers, is remeshed
// at 2 mm.
func Example_mesh_remesh() {
	var src strings.Builder
	src.WriteString("v 0 0 3\nv 0 0 -3\n")
	for i := range 12 {
		a := 2 * math.Pi * float64(i) / 12
		fmt.Fprintf(&src, "v %.6f %.6f 0\n", 5*math.Cos(a), 5*math.Sin(a))
	}
	for i := range 12 {
		j := (i + 1) % 12
		fmt.Fprintf(&src, "f 1 %d %d\nf 2 %d %d\n", 3+i, 3+j, 3+j, 3+i)
	}
	mesh, err := decad.ReadOBJ(context.Background(), strings.NewReader(src.String()))
	if err != nil {
		fmt.Printf("failed to read mesh: %s\n", err)
		return
	}
	even, err := decad.RemeshMesh(context.Background(), mesh, units.Millimeters(2))
	if err != nil {
		fmt.Printf("failed to remesh: %s\n", err)
		return
	}
	fmt.Printf("triangles: %d -> %d, closed: %t\n", mesh.Report().Triangles, even.Report().Triangles, even.Report().Closed)
	// Output:
	// triangles: 24 -> 70, closed: true
}
