#pragma once

#include <list>
#include <algorithm>
#include "Collision.h"
#include "WorldMaths.h"

// Tree data strructure created for the purpose of broad phase detection
// Following this guide https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials/6accelerationstructures/Physics%20-%20Spatial%20Acceleration%20Structures.pdf

template<class T>
struct OcTreeEntry {
	Vec3f position;
	Vec3f size;
	T object;

	OcTreeEntry(T obj, Vec3f pos, Vec3f size)
		: object(obj), position(pos), size(size) {
	}
};

template<class T>
class OcTreeNode {
public:
	std::vector<OcTreeEntry<T>> contents;

	// Center of the node.
	Vec3f position;
	// Distances between position and edge of node.
	Vec3f size;

	// Array pointer to 0 or 8 children
	OcTreeNode<T> *children = nullptr;

public:
	OcTreeNode() {	};
	OcTreeNode(Vec3f pos, Vec3f size) 
		: position(pos), size(size) {
	}
	~OcTreeNode() {
		if(children != nullptr) delete[] children;
	};

	void split() {
		Vec3f childPos;

		Vec3f childSize;
		Vec3f::scale(&childSize, &size, 0.5f);
		children = new OcTreeNode<T>[8];

		// Order of split nodes is from top-left to bottom-right. Front then Back.

		// Top-Left
		childPos.set(position.x - childSize.x, position.y + childSize.y, position.z + childSize.z);
		children[0] = OcTreeNode<T>(childPos, childSize);
		// Top-Right
		childPos.set(position.x + childSize.x, position.y + childSize.y, position.z + childSize.z);
		children[1] = OcTreeNode<T>(childPos, childSize);
		// Bottom-Left
		childPos.set(position.x - childSize.x, position.y - childSize.y, position.z + childSize.z);
		children[2] = OcTreeNode<T>(childPos, childSize);
		// Bottom-Right
		childPos.set(position.x + childSize.x, position.y - childSize.y, position.z + childSize.z);
		children[3] = OcTreeNode<T>(childPos, childSize);


		// Top-Left
		childPos.set(position.x - childSize.x, position.y + childSize.y, position.z - childSize.z);
		children[4] = OcTreeNode<T>(childPos, childSize);
		// Top-Right
		childPos.set(position.x + childSize.x, position.y + childSize.y, position.z - childSize.z);
		children[5] = OcTreeNode<T>(childPos, childSize);
		// Bottom-Left
		childPos.set(position.x - childSize.x, position.y - childSize.y, position.z - childSize.z);
		children[6] = OcTreeNode<T>(childPos, childSize);
		// Bottom-Right
		childPos.set(position.x + childSize.x, position.y - childSize.y, position.z - childSize.z);
		children[7] = OcTreeNode<T>(childPos, childSize);
	}

	void insert(T &object, const Vec3f &objectPos, const Vec3f &objectsize, int depthLeft, int maxSize) {
		if(! CollisionDetect::broadCheck(objectPos, objectsize, position, size)) {
			return;
		}

		// Inserts object at the lowest current depth.
		if(children != nullptr) {
			for(int i = 0; i < 8; i++) {
				children[i].insert(object, objectPos, objectsize, depthLeft - 1, maxSize);
			}
		}
		else {
			contents.emplace_back(object, objectPos, objectsize);
			// Split the LEAF node into 8 children if maximum limit has been reached.
			if(contents.size() > maxSize && depthLeft > 0) {
				if(children == nullptr) {
					split();

					for(const OcTreeEntry<T> &elem : contents) {
						for(int j = 0; j < 8; j++) {
							OcTreeEntry<T> entry = elem;
							children[j].insert(entry.object, entry.position, entry.size, depthLeft - 1, maxSize);
						}
					}

					// Pointers have been moved to children so current node is cleared.
					contents.clear();
				}
			}
		}

	};

	std::vector<OcTreeNode<T>*> getAllLeafNodes() {
		std::vector<OcTreeNode<T>*> leafNodes;

		if(!children) {
			leafNodes.push_back(this);
		}
		else {
			for(int i = 0; i < 8; i++) {
				std::vector<OcTreeNode<T>*> temp;
				temp = children[i].getAllLeafNodes();
				leafNodes.insert(leafNodes.end(), temp.begin(), temp.end());
			}
		}

		return leafNodes;
	}

};

template<class T>
class OcTree {
protected:
	OcTreeNode<T>* root;
	int maxDepth;
	int maxSize;

public:
	OcTree(Vec3f initialPos, Vec3f size, int maxDepth = 6, int maxSize = 5)
		: maxDepth(maxDepth), maxSize(maxSize) {
		root = new OcTreeNode<T>(initialPos, size);
	};

	~OcTree() {	};

	void insert(T object, const Vec3f &pos, const Vec3f &size) {
		root->insert(object, pos, size, maxDepth, maxSize);
	};

	std::vector<OcTreeNode<T>*> getAllLeafNodes() const {
		std::vector<OcTreeNode<T>*> nodes;

		OcTreeNode<T>* currentNode = root;

		nodes = root->getAllLeafNodes();

		return nodes;
	}

	// Finding combinations of pairs of objects were made following this guide.
	// https://stackoverflow.com/questions/1876474/c-newbie-needs-helps-for-printing-combinations-of-integers

	/* Given an array of elements, the output will be filled with all combinations of the input.
	All elements will be combined into pairs of 2 by default. 
	(First call can alter parameter based on guidelines below).
	@param r, start_r : Must be equal to each other.
	@param initial : Must always be equal to 0.
	@param input : Must have at least 2 elements.
	*/
	void getCollisionPairs(const std::vector<OcTreeEntry<T>> &input, std::vector<T> &output, T temp[], int r = 2, int initial = 0, const int start_r = 2) {

		// Recursive exit.
		if(r == 0) {
			for(int j = start_r; j > 0; j--) {
				output.push_back(temp[j-1]);
			}
			return;
		}

		int n = input.size();

		for(int i = initial; i < n; i++) {
			temp[r-1] = input[i].object;
			getCollisionPairs(input, output, temp, r - 1, i + 1);
		}

	};

private:

};
