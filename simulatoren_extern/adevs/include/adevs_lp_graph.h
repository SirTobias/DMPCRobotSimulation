/**
 * Copyright (c) 2013, James Nutaro
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies, 
 * either expressed or implied, of the FreeBSD Project.
 *
 * Bugs, comments, and questions can be sent to nutaro@gmail.com
 */
#ifndef adevs_lp_graph_h
#define adevs_lp_graph_h
#include <vector>
#include <map>

namespace adevs
{

/**
 * This is a graph for describing how processors in a 
 * parallel simulation are connected to each other.
 * There should be a directed edge from processor A
 * to processor B if a model assigned to A sends output
 * to a model assigned to B.
 */
class LpGraph
{
	public:
		/// Create a graph without any edges
		LpGraph():nodes(0){}
		/// Create an edge from node A to node B
		void addEdge(int A, int B)
		{
			if (E.find(A) == E.end()
					&& I.find(A) == I.end())
				nodes++; 
			if (E.find(B) == E.end()
					&& I.find(B) == I.end())
				nodes++; 
			E[A].push_back(B);
			I[B].push_back(A);
		}
		/// Get the number of LPs
		int getLPCount() const { return nodes; }
		/// Get the influencers of node B
		const std::vector<int>& getI(int B) { return I[B]; }
		/// Get the influencees of node A
		const std::vector<int>& getE(int A) { return E[A]; }
		/// Destructor
		~LpGraph(){}
	private:
		// Number of nodes in the graph
		int nodes;
		// Influencee graph
		std::map<int,std::vector<int> > E;
		// Complimentary influencer graph
		std::map<int,std::vector<int> > I;
};

}

#endif
