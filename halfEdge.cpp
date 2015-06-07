/*
 * halfEdge.cpp
 * 
 * Contains all of the code for creating a half edge structure as well as
 * computing the normals of the vertices from said structure.
 */

#include "halfEdge.h"

// Uses the Cantor pairing function to return a unique key
int findKey(int a, int b)
{
    return ((a + b)*(a + b + 1)/2) + b;
}

// Function to load the points into the halfVertices vector.
void createHalfVertices(vector<Vector3d> *pts, vector<HEV*> *halfVertices)
{
    for (int i = 0; i < pts->size(); i++)
    {
        HEV *v = new HEV;
        Vector3d pt = pts->at(i);
        v->x = pt(0);
        v->y = pt(1);
        v->z = pt(2);

        halfVertices->push_back(v);
    }
}

// Function to load the faces into the halfFaces and halfEdges vectors.
void createHalfOthers(vector<Vector3d> *f, vector<HE*> *halfEdges, vector<HEF*> *halfFaces)
{
    map<int, HE*> vertices;
    map<int, HE*>::iterator it;

    for (int i = 0; i < f->size(); i++)
    {
        // Extract the vertex indices from the face
        Vector3d fa = f->at(i);
        int ind1 = fa(0);
        int ind2 = fa(1);
        int ind3 = fa(2);

        HE *e1 = new HE;
        HE *e2 = new HE;
        HE *e3 = new HE;
        HEF *face = new HEF;

        // Default the HE brother and vertex pointers to null
        e1->brother = e2->brother = e3->brother = NULL;
        e1->vertex = e2->vertex = e3->vertex = NULL;

        // Default the next pointers to clockwise; the final orientation will
        // be handled elsewhere
        e1->next = e2;
        e2->next = e3;
        e3->next = e1;
        
        // Set the adjacent face of the half edges to the half edge face
        e1->adj_face = e2->adj_face = e3->adj_face = face;

        // Point the face to one of the half edges
        face->adj = e1;
        // mark the face as unoriented
        face->oriented = 0;
        // Add the vertex indices to the face
        face->a = ind1;
        face->b = ind2;
        face->c = ind3;

        // Calculate map keys for the three edges
        int key1 = findKey(min(ind1, ind2), max(ind1, ind2));
        int key2 = findKey(min(ind2, ind3), max(ind2, ind3));
        int key3 = findKey(min(ind3, ind1), max(ind3, ind1));

        // See if an edge if already in the map with key1
        it = vertices.find(key1);
        if(it == vertices.end())
        {
            // If there is no match for key1, then add e1 to the map
            vertices.insert(pair<int, HE*>(key1, e1));
        }
        else
        {
            // If there is a match, then set e1's brother to the edge in the map,
            // and the map edge's brother to e1
            e1->brother = vertices[key1];
            vertices[key1]->brother = e1;
        }

        // Repeat for edges e2 and e3
        it = vertices.find(key2);
        if(it == vertices.end())
        {
            vertices.insert(pair<int, HE*>(key2, e2));
        }
        else
        {
            e2->brother = vertices[key2];
            vertices[key2]->brother = e2;
        }

        it = vertices.find(key3);
        if(it == vertices.end())
        {
            vertices.insert(pair<int, HE*>(key3, e3));
        }
        else
        {
            e3->brother = vertices[key3];
            vertices[key3]->brother = e3;
        }

        // Add the half edges to the halfEdges vector
        halfEdges->push_back(e1);
        halfEdges->push_back(e2);
        halfEdges->push_back(e3);
        
        // Add the face to the halfFaces vector
        halfFaces->push_back(face);
    }
}

// Orients all of the faces. If the function fails to orient all faces properly
// then it will return a non-zero value; otherwise it returns 0.
int orientFace(HEF *hef, vector<HEF*> *halfFaces, vector<HEV*> *halfVertices)
{
    int successful = 0;

    // Check to see if this face is oriented. If it isn't, orient it and its
    // neighbors; otherwise, simply return successful.
    if (hef->oriented == 0)
    {
        // Get the three edges surrounding this face for orientation purposes
        // and also to get the adjacent faces
        HE *temp1 = hef->adj;
        HE *temp2 = temp1->next;
        HE *temp3 = temp2->next;

        // Set the edges to point at the face vertices, default clockwise.
        temp1->vertex = halfVertices->at(hef->b);
        temp2->vertex = halfVertices->at(hef->c);
        temp3->vertex = halfVertices->at(hef->a);

        // Set the face vertices to point at the half edge that leaves them
        halfVertices->at(hef->a)->out = temp1;
        halfVertices->at(hef->b)->out = temp2;
        halfVertices->at(hef->c)->out = temp3;

        // Get the edge brothers
        HE *temp1Bro = temp1->brother;
        HE *temp2Bro = temp2->brother;
        HE *temp3Bro = temp3->brother;

        // Get the adjacent faces
        // Only retrieve a face if brother is not null
        vector<HEF*> *tempf = new vector<HEF*>; //Store adjacent faces
        vector<HE*> *tempb = new vector<HE*>;  //Store the respective brother edge
        vector<HE*> *temp = new vector<HE*>;   //Store the respective half edge
        if (temp1Bro != NULL)
        {
            tempf->push_back(temp1Bro->adj_face);
            tempb->push_back(temp1Bro);
            temp->push_back(temp1);
        }
        if (temp2Bro != NULL)
        {
            tempf->push_back(temp2Bro->adj_face);
            tempb->push_back(temp2Bro);
            temp->push_back(temp3);
        }
        if (temp3Bro != NULL)
        {
            tempf->push_back(temp3Bro->adj_face);
            tempb->push_back(temp3Bro);
            temp->push_back(temp3);
        }

        bool swappedOnce = false;
        // Loop through the adjacent faces
        for(int i = 0; i < tempf->size(); i++)
        {
            // Check if the current adjacent face has been oriented. If not,
            // ignore it.
            if(tempf->at(i)->oriented == 1)
            {
                // Check to see if the face's half edge and the brother edge
                // point to the same vertex. If they do, flip the orientation
                // of the current face and flip the swappedOnce variable.
                // If swappedOnce is already true and two brother edges point
                // to the same vertex, then we have an invalid orientation.
                if(!swappedOnce && tempb->at(i)->vertex == temp->at(i)->vertex)
                {
                    HEV * tv1 = temp1->vertex;
                    HEV * tv2 = temp2->vertex;
                    HEV * tv3 = temp3->vertex;
                    temp1->vertex = tv3;
                    temp2->vertex = tv1;
                    temp3->vertex = tv2;
                    temp1->next = temp3;
                    temp2->next = temp1;
                    temp3->next = temp2;
                    // Change the incident vertices to point at the half edges
                    // pointing away from them
                    halfVertices->at(hef->a)->out = temp3;
                    halfVertices->at(hef->b)->out = temp1;
                    halfVertices->at(hef->c)->out = temp2;
                    swappedOnce = !swappedOnce;
                }
                // Invalid orientation!
                else if(swappedOnce && tempb->at(i)->vertex == temp->at(i)->vertex)
                {
                    successful++;
                }
            }
        }

        hef->oriented = 1;

        // Now that our current face is oriented, loop through the adjacent
        // faces again and call orientFace on the ones that have not been
        // oriented
        for(int i = 0; i < tempf->size(); i++)
        {
            if(tempf->at(i)->oriented == 0)
            {
                successful += orientFace(tempf->at(i), halfFaces, halfVertices);
            }
        }
    }

    return successful;
}

// Function to check whether or not brother edges were assigned correctly
int checkBrothers(int select, vector<HE*> *halfEdges)
{
    int counter = 0;
    for (int i = 0; i < halfEdges->size(); i++)
    {
        HE * e = halfEdges->at(i);
        if (e->brother != NULL && e->vertex == e->brother->vertex && select == 0)
            counter++;
        if(e->brother != NULL && e != e->brother->brother && select == 1)
            counter++;
        if(e->brother == NULL && select == 2)
            counter++;
    }
    return counter;
}

// Calculates and returns the normal of the given face
Vector3d findFaceNormal(HEF *hef, vector<HEV*> *halfVertices)
{
    HEV *temp1 = halfVertices->at(hef->a);
    HEV *temp2 = halfVertices->at(hef->b);
    HEV *temp3 = halfVertices->at(hef->c);

    Vector3d v1(temp1->x, temp1->y, temp1->z);
    Vector3d v2(temp2->x, temp2->y, temp2->z);
    Vector3d v3(temp3->x, temp3->y, temp3->z);

    Vector3d vec1 = v2 - v1;
    Vector3d vec2 = v3 - v1;

    Vector3d norm = vec1.cross(vec2);
    norm.normalize();
    
    return norm;
}

// Calculates and returns the area of the given face. The face is assumed to be
// a triangle.
double findFaceArea(HEF *hef, vector<HEV*> *halfVertices)
{
    HEV *temp1 = halfVertices->at(hef->a);
    HEV *temp2 = halfVertices->at(hef->b);
    HEV *temp3 = halfVertices->at(hef->c);

    Vector3d v1(temp1->x, temp1->y, temp1->z);
    Vector3d v2(temp2->x, temp2->y, temp2->z);
    Vector3d v3(temp3->x, temp3->y, temp3->z);

    double sideA = distance(v2, v1);
    double sideB = distance(v3, v2);
    double sideC = distance(v1, v3);
    double s = (sideA + sideB + sideC) / 2.0;

    return sqrt(s * (s - sideA) * (s - sideB) * (s - sideC));
}

// Returns a vector of all the faces incident to the given vertex.
vector<HEF*> * findIncidentFaces(HEV *hev)
{
    vector<HEF*> *incFaces = new vector<HEF*>;
    HEF *face1 = hev->out->adj_face;
    incFaces->push_back(face1);

    HE *tempEdge = hev->out->brother;
    HEF *tempFace = NULL;
    if (tempEdge != NULL)
        tempFace = tempEdge->adj_face;

    while (tempFace != NULL && tempFace != face1)
    {
        incFaces->push_back(tempFace);

        tempEdge = tempEdge->next->brother;
        if (tempEdge != NULL)
        {
            tempFace = tempEdge->adj_face;
        }
        else
        {
            tempFace = NULL;
        }
    }

    // if tempFace becomes null, then we hit a boundary somewhere and have not
    // accounted for all the faces around the vertex.
    if (tempFace == NULL)
    {
        // Start again from face1, but rotate around the vertex in the opposite
        // direction
        tempEdge = hev->out->next->next->brother;
        if (tempEdge != NULL)
            tempFace = tempEdge->adj_face;
        else
            tempFace = NULL;

        while (tempFace != NULL && tempFace != face1)
        {
            incFaces->push_back(tempFace);

            tempEdge = tempEdge->next->next->brother;
            if (tempEdge != NULL)
                tempFace = tempEdge->adj_face;
            else
                tempFace = NULL;
        }
    }

    return incFaces;
}

// Returns the normal vector of the given vertex using area weighted normals.
Vector3d findVertexNormal(HEV *hev, vector<HEV*> *halfVertices)
{
    vector<HEF*> *incidentFaces = findIncidentFaces(hev);

    Vector3d norm(0, 0, 0);

    for(int i = 0; i < incidentFaces->size(); i++)
    {
        Vector3d faceNorm = findFaceNormal(incidentFaces->at(i), halfVertices);

        double area = findFaceArea(incidentFaces->at(i), halfVertices);

        faceNorm = faceNorm + area;
        norm = norm + faceNorm;
    }

    norm.normalize();
    
    return norm;
}

// Finds the normal vector for every vertex in halfVertices and stores the
// normals in the global normals vector with index corresponding to its
// respective vertex.
void findAllVertexNormals(vector<Vector3d> * normals, vector<HEV*> *halfVertices)
{
    for(int i = 0; i < halfVertices->size(); i++)
    {
        Vector3d normal = findVertexNormal(halfVertices->at(i), halfVertices);
        normals->push_back(normal);
    }
}

// Uses the calculated normal vectors to find the bump map normals by
// multiplying each normal by a random value between 0 and 1.
void calcBumpNormals(vector<Vector3d> *normals, vector<Vector3d> *bumpNormals)
{
    for(int i = 0; i < normals->size(); i++)
    {
        double r = (double) rand() / (double) RAND_MAX;
        Vector3d bumpNormal = normals->at(i) * r;
        bumpNormals->push_back(bumpNormal);
    }
}
