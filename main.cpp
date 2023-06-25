#include <iostream>
#include <vector>
#include <stack>
#include <queue>

using namespace std;

template <typename G, typename L>
class Graph
{
private:
    int n;
    vector<vector<G>> adj;
    vector<vector<pair<G,L>>> adj1;

    void depFirSrh(vector<G> &dfs, vector<G> &vis, G node)
    {
        vis[node] = 1;
        dfs.push_back(node);
        for(auto it : adj[node])
        {
            if(!vis[it])
            {
                depFirSrh(dfs,vis,it);
            }
        }
    }

    bool checkCycleInDirected(vector<vector<G>> &adj, vector<G> &vis, G node)
    {
        vis[node] = 1;
        for(auto it : adj[node])
        {
            if(!vis[it]){
                if(checkCycleInDirected(adj,vis,it)) 
                return true;
            }
            else if(vis[it] == 1)
            {
                return true;
            }
        }
        vis[node] = 2;
        return false;
    }

    void bridges(vector<G> &dis, vector<G> &low, vector<G> &vis, vector<pair<G,G>> &ans, G node, int &count)
    {
        vis[node] = 1;
        dis[node] = low[node] = count++;
        for(auto it : adj[node])
        {
            if(vis[it]==0)
            {
                bridges(dis,low,vis,ans,it,count);
                low[it] = min(low[it],low[node]);
                if(low[it]>dis[node])
                {
                    ans.push_back({node,it});
                }
            }
            else {
                low[node] = min(low[node],dis[it]);
            }
        }
    }
    
    void articulation(vector<G> &dis, vector<G> &low, vector<G> &vis, vector<G> &ans, G node, G parent, int &count)
    {
        vis[node] = 1;
        dis[node] = low[node] = count++;
        G child = 0;
        for(auto it : adj[node])
        {
            if(vis[it]==0)
            {
                articulation(dis,low,vis,ans,it,node,count);
                low[it] = min(low[it],low[node]);
                if(low[it]>=dis[node] && parent!=-1)
                {
                    ans.push_back(node);
                }
                child++;
            }
            else {
                low[node] = min(low[node],dis[it]);
            }
        }
        if(child>1 && parent==-1)
        {
            ans.push_back(node);
        }
    }


public:
    Graph(vector<vector<G>> adj, int n){
        this->n = n;
        this->adj = adj;
    }
   
    vector<G> breadthFirstSearch(G node = 0){
        vector<G> bfs;
        vector<G> vis;
        queue<G> q;
        q.push(node);
        vis[node] = 1;
        while(!q.empty()){
        G curr = q.front();
        bfs.push_back(curr);
        q.pop();
        for(auto it : adj[curr]){
            if(!vis[it]){
                q.push(it);
                vis[it] = 1;
                }
            }
        }
        return bfs;
    }

    vector<G> depthFirstSearch(G node = 0){
        vector<G> dfs;
        vector<G> vis;
        for(int i=0;i<n;i++)
        {
            if(!vis[i])
            {
                depFirSrh(dfs,vis,i);
            }
        }
        return dfs;
    }

    // Applicable only for directed graph and graph must not contain cycle.
    vector<G> toposort(){
        vector<G> topo;
        vector<G> indegree(n,0);
        stack<G> st;

        for(int i=0;i<n;i++)
        {
            for(auto it : adj[i])
            {
                indegree[it]++;
            }
        }
        for(int i=0;i<n;i++)
        {
            if(indegree[i] == 0)
            st.push(i);
        }
        while(!st.empty()){
            G curr = st.top();
            topo.push_back(curr);
            st.pop();
            for(auto it : adj[curr])
            {
                indegree[it]--;
                if(indegree[it] == 0)
                {
                    st.push(it);
                }
            }
        }
        if(topo.size()!=n) return {};
        return topo;
    }

    bool detectCycleInUndirectedGraph(G node = 0){
        queue<pair<G,G>> q;
        vector<G> vis(n,0);
        q.push({node,-1});
        while(!q.empty())
        {
            G curr = q.front().first;
            G parent = q.front().second;
            q.pop();
            for(auto it : adj[curr])
            {
                if(!vis[it]){
                    q.push({it,node});
                    vis[it] = 1;
                }
                else if(it!=parent){
                    return true;
                }
            }
        }
        return false;
    }

    bool detectCycleInDirectedGraph(vector<vector<G>> &adj) {
        vector<G> vis(n,0);
        for(int i=0;i<n;i++)
        {
            if(!vis[i])
            {
                if(checkCycleInDirected(adj,vis,i)) return true;
            }
        }
        return false;
    }

    bool checkBipartiteGraph(G node = 0){
        queue<G> q;
        vector<G> color(n,0);
        vector<G> vis(n,0);
        vis[node] = 1;
        color[node] = 1;
        q.push(node);
        while(!q.empty())
        {
            G curr = q.front();
            q.pop();
            for(auto it : adj[curr])
            {
                if(!vis[it])
                {
                    q.push(it);
                    vis[it] = 1;
                    color[it] = 1-color[curr];
                }
                else if(color[it]==color[curr])
                {
                    return false;
                }
            }
        }
        return true;
    }

    vector<G> dijastra(G node = 0){
        vector<G> dist(n,INT_MAX);
        priority_queue<pair<G,G>, vector<pair<G,G>>, greater<pair<G,G>>> pq;
        pq.push({0,node});
        while(!pq.empty()){
            G curr = pq.top().first;
            L wt = pq.top().second;
            pq.pop();
            if(dist[curr]<wt) continue;

            for(auto it : adj[curr]){
                if(dist[it]>dist[curr]+wt){
                    dist[it] = dist[curr]+wt;
                    pq.push({dist[it],it});
                }
            }
        }
        return dist;
    }

    vector<G> bellmanFord(G node = 0){
        G n = adj.size();
        vector<G> dist(n,INT_MAX);
        for(int i=0;i<n-1;i++){
            for(int j=0;j<adj.size();j++){
                G u = j;
                for(auto it : adj[j]){
                    G v = adj[it].first;
                    L wt = adj[it].second;
                    if(dist[v]>dist[u]+wt){
                        dist[v] = dist[u]+wt;
                    }
                }
            }
        }
        for(int j=0;j<adj.size();j++){
            int u = j;
            for(auto it : adj[j]){
                int v = adj[it].first;
                int wt = adj[it].second;
                if(dist[v]>dist[u]+wt){
                    return {-1};
                }
            }
        }
        return dist;
    }

    vector<vector<int>> flloydWarshel(){
        
    }

    G findParent(vector<G> &parent, G node){
        if(parent[node] == node)
        {
            return node;
        }
        return parent[node] = findParent(parent,parent[node]);
    }

    void unionFind(G u, G v, vector<G> &parent, vector<G> &rank){
        G pu = findParent(parent,u);
        G pv = findParent(parent,v);

        if(rank[pu] < rank[pv])
            parent[pu] = pv;
        else if(rank[pv] < rank[pu])
            parent[pv] = pu;
        else{
            parent[pv] = pu;
            rank[pu]++;
        }
    }
    // There are two algo for finding Minimun Spanning tree 
    // 1. Prism algo
    // 2. Kruskals algo 

    void prism_Algo(){
        priority_queue<pair<G,G>, vector<pair<G,G>>, greater<pair<G,G>>> pq;
        vector<G> key(n,INT_MAX);
        vector<bool> mst(n,false);
        pq.push({0,0});

        for(int c=0;c<n-1;c++){
            G node = pq.top().second;
            pq.pop();
            mst[node] = true;
            for(auto it : adj1[node]){
                G curr = it.first;
                L wt = it.second;
                if(mst[curr]==false && wt<key[curr]){
                    pq.push({wt,curr});
                    key[curr] = wt;
                }
            }
        }
    }

    void kurskals_Algo(){
        vector<pair<L,pair<G,G>>> edges;
        vector<G> parent(n,-1);
        vector<G> rank(n,0);
        L cost = 0;
        for(int i=0;i<n;i++){
            for(auto it : adj1[i]){
                edges.push_back({it.second,{i,it.first}});
            }
        }
        for(int i=0;i<n;i++){
            parent[i] = i;
        }

        sort(edges.begin(),edges.end());

        for(auto it : edges){
            if(findParent(parent,it.second.first)!=findParent(parent,it.second.second)){
                cost += it.first;
                unionFind(it.second.first,it.second.second,parent,rank);
            }
        }
    }

    vector<pair<G,G>> bridgesInGraph(G node = 0){
        vector<G> dis(n,0);
        vector<G> low(n,0);
        vector<G> vis(n,0);
        vector<pair<G,G>> ans;
        for(int i=0;i<n;i++)
        {
            if(!vis[i])
            {
                bridges(dis,low,vis,ans,i,0);
            }
        }
        return ans;
    }

    vector<G> articulationPoint(){
        vector<G> dis(n,0);
        vector<G> low(n,0);
        vector<G> vis(n,0);
        vector<G> ans;
        for(int i=0;i<n;i++)
        {
            if(!vis[i])
            {
                articulation(dis,low,vis,ans,i,-1,0);
            }
        }
        return ans;
    }

};

