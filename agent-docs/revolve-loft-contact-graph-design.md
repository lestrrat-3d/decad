# Revolve and LoftFaces contact graph

This private graph names source events shared by a finite Revolve boundary and
an authenticated, cone-trimmed `LoftFaces` body. The producer is the current
`certifyRevolveLoftFacesSource` certificate and its complete face-pair census.
The graph does not establish a closed joined boundary or admit `Union`.

## Source ownership

The graph has one entry for every ordered pair of live Revolve and trimmed
`LoftFaces` faces. It also has one entry for every source cell in the census
manifest, including strict-side cells on pair misses and vertical cells with
no retained wall. A retained cell belongs to exactly one pair. Its identity
includes the live face, `FeatureRef`, source segment and span, LoftFaces band,
and exact source and band-time intervals. Miss cells own no seam station ID.

An occupied heel-cap arc has one pair entry and two distinct endpoints. Each
endpoint must have the unique source-side cell proved by the heel-arc
side-tie check against the graph's own census. A whole radial Sketch arc contact has one
complete station run. All cap endpoints and stations receive IDs from one
counter, so unrelated event kinds cannot share an ID. No ID is inferred from
nearby held coordinates.

## Held anchors and freshness

The checked open-patch constructor uses the live graph, selects one whole-arc
run by exact source interval and live face, and checks each station of the
run, in order, against the actual seam index in both open patches. It checks
the shared held point, each side's vertex bound and that the run's station IDs
are distinct. The patches carry no IDs; the run keeps them.

The cap-ring blank-shell constructor selects a graph run and its occupied cap
arcs. After building, it binds every station ID to the actual shell seam index
and compares that point with both checked open patches. Each cap endpoint ID
must name exactly one ring vertex and retain its unique tied side-cell ID.
The ring's held point and blank-side bound are checked. Duplicate or missing
shell, patch, ring, or side-cell anchors refuse.

The multi-run blank-shell consumer inserts every graph whole-arc run and every
occupied cap arc into one closed Revolve shell. Each cut must lie strictly
inside its authenticated meridian section. Cuts on one section are sorted by
their exact meridian fractions; equal fractions refuse. One angular grid is
shared by every ring. Stations from different runs may use the same exact ray
and still name different held vertices on their own rings. Two stations from
one run or a cap endpoint and a station on one ray refuse. Every station and
cap source box must stay in strict angular order beside its neighboring rays.

The constructor binds every station ID to its shell vertex and both checked
open-patch vertices, and binds each cap endpoint ID to its ring vertex and
live side cell. Each shell triangle records its Revolve section, adjacent
ring band, angular ray interval and triangle half. The shell repeats the
orientation, crossing, Euler, positive-volume and face-bound audits. These
records do not classify a triangle as retained or removed.

## Vertical source contact on the blank shell

The vertical-trace consumer builds the live graph and all-run shell. It
selects one constant-X source cell by its graph ID and finds the cap arc with
that cell's section and blank face. The graph's two endpoint-cell IDs choose
the path ends. Exact mapped source endpoints link every retained piece on
that planar side; a missing piece, extra component, changed root bracket, or
duplicate owner refuses.

The supported path uses exact mapped two-control lines and live Sketch arcs
whose positive similarity maps their center to the Revolve axis. Every line
and arc has a strict angular derivative in the positive radial half-plane.
Lines adjacent to an arc must leave its circle radially; nonadjacent lines
must have disjoint exact control hulls. Each piece lies inside the cap circle
except at its graph-owned root bracket. Other curve and path classes refuse.

The consumer finds each piece's crossing with every shell ray. A ray may meet
different path pieces more than once, including a cap endpoint and the inner
arc. Exact source parameters bound line crossings; centered-arc radius and
ray direction bound arc crossings. Each adjacent pair of held path vertices
must fit one ray wedge. The line segment's endpoint boxes and the arc chord's
angle bound charge source-to-held displacement; a bound at or above 0.1 mm
refuses.

Every affected planar fan wedge is split by its ordered radial contacts and
source edges. Exact edge-crossing checks and ear clipping triangulate each
bounded region. Each emitted triangle records one original section, ring
band, and ray wedge. The rebuilt shell repeats orientation, Euler, exact
facet-crossing, positive-volume and face-bound audits. The trace does not
classify retained LoftFaces faces or produce a joined Union boundary.

A stored graph is checked against the live graph before later use. The
comparison includes every pair verdict, cell owner and exact interval,
complete station run, and occupied cap arc. A changed source station, face,
reference, interval, root, or event ID refuses. Graph records
and the open patches remain construction pieces; the retained LoftFaces faces,
joined shell, boundary displacement, and occupied-volume proofs are separate.

A top-level consumer (the all-run shell, the cap-ring shell, the vertical
trace and the clear-patch context) opens one call scope. Inside it, the source
certificate and contact graph of a body pair are built from the live bodies the
first time a check needs them, and every later check in the same call reuses
them. A census is derived once per source certificate in the call. Nothing
inside a call edits the bodies, so a rebuild inside the call would produce the
same records. Separate top-level calls never share a scope, so a body changed
between calls is certified again and a changed record refuses.

## Clear blank-shell triangles

A clear-patch consumer constructs the graph and all-run shell from the current
Revolve and trimmed `LoftFaces` producers, then tessellates the actual trimmed
tool at `VerifyAll` with the caller's chord tolerance. A candidate triangle
must retain its source section, face origins, ring band, angular interval,
held vertices and sufficient face bound. The tool mesh must publish an embedded
boundary and an occupied-volume proof. A missing or changed producer,
provenance record, bound or proof leaves the candidate unclassified.
The source certificate records the trimmed body's faceted proof data by value.
The consumer compares its `VerifyAll` mesh with a fresh payload restatement,
then copies the audited shell and mesh into its private classification context.
Candidate distance and point queries read only those copies. Before each
single-triangle classification, the consumer compares the complete proof data
of both copies with sealed values and rechecks the source records and live tool
payload. A changed body or context cannot borrow an earlier proof.

For every tool facet, exact triangle-distance predicates must prove a gap
strictly greater than the sum of the shell's complete face bound and the
tool's mesh bound. A candidate beside a contact run or trim-cap arc generally
cannot pass this test and stays unclassified. The exact interior centroid is
rounded with its displacement charged against the gap. A verified point query
must classify that point as inside or outside the trimmed tool. The gap then
keeps the tool boundary away from every point of the connected shell triangle,
so that one point's occupancy labels the whole triangle. An undecided or
boundary point query leaves it unclassified. These labels do not select a
retained mask, build the LoftFaces side, close a joined shell or admit `Union`.
