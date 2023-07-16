#include <iostream>
#include <vector>
#include <stack>
#include <queue>
// #include <pair>

using namespace std;

template <typename G, typename L>
class Graph
{
private:
    int n;
    vector< vector<G> > adj;
    vector< vector< pair<G,L> > > adj1;

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

    bool checkCycleInDirected(vector< vector<G> > &adj, vector<G> &vis, G node)
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

    void bridges(vector<G> &dis, vector<G> &low, vector<G> &vis, vector< pair<G,G> > &ans, G node, G parent, int &count)
    {
        vis[node] = 1;
        dis[node] = low[node] = count++;
        for(auto it : adj[node])
        {
            if(parent==it) continue;
            if(vis[it]==0)
            {
                bridges(dis,low,vis,ans,it,node,count);
                low[node] = min(low[it],low[node]);
                if(low[it]>dis[node])
                {
                    ans.push_back(make_pair(node,it));
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
                low[node] = min(low[it],low[node]);
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
    Graph(vector< vector<G> > adj, int n, vector<L>&wt, int itr = 0){
      
        this->n = n;
        adj1.resize(n);
        this->adj.resize(n);
        for(int i=0;i<adj.size();i++)
        {
            this->adj[adj[i][0]].push_back(adj[i][1]);
            adj1[adj[i][0]].push_back(make_pair(adj[i][1],wt[i]));
            if(itr == 1)
            {
                this->adj[adj[i][1]].push_back(adj[i][0]);
                adj1[adj[i][1]].push_back(make_pair(adj[i][0],wt[i]));
            }
        }
    }
   
    vector<G> breadthFirstSearch(G node = 0){
        vector<G> bfs;
        vector<G> vis(n,0);
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
        vector<G> vis(n,0);
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
        if(topo.size()!=n) {
            vector<int> empty;
            return empty;
        }
        return topo;
    }

    bool detectCycleInUndirectedGraph(G node = 0){
        queue< pair<G,G> > q;
        vector<G> vis(n,0);
        q.push(make_pair(node,-1));
        while(!q.empty())
        {
            G curr = q.front().first;
            G parent = q.front().second;
            q.pop();
            for(auto it : adj[curr])
            {
                if(!vis[it]){
                    q.push(make_pair(it,curr));
                    vis[it] = 1;
                }
                else if(it!=parent){
                    return true;
                }
            }
        }
        return false;
    }

    bool detectCycleInDirectedGraph() {
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
        priority_queue< pair<G,G>, vector< pair<G,G> >, greater< pair<G,G> > > pq;
        pq.push(make_pair(0,node));
        dist[node] = 0;
        while(!pq.empty()){
            G curr = pq.top().second;
            L wt = pq.top().first;
            pq.pop();
            if(dist[curr]<wt) continue;

            for(auto it : adj1[curr]){
                if(dist[it.first]>dist[curr]+it.second){
                    dist[it.first] = dist[curr]+it.second;
                    pq.push(make_pair(dist[it.first],it.first));
                }
            }
        }
        return dist;
    }

    vector<G> bellmanFord(G node = 0){
        G n = adj1.size();
        vector<G> dist(n,INT_MAX);
        dist[node] = 0;
        for(int i=0;i<n-1;i++){
            for(int j=0;j<adj1.size();j++){
                G u = j;
                for(auto it : adj1[j]){
                    G v = it.first;
                    L wt = it.second;
                    if(dist[v]>dist[u]+wt){
                        dist[v] = dist[u]+wt;
                    }
                }
            }
        }
         
        for(int j=0;j<adj1.size();j++){
            int u = j;
            for(auto it : adj1[j]){
                int v = it.first;
                int wt = it.second;
                if(dist[v]>dist[u]+wt){
                    vector<int>ans(1,-1);
                    return ans;
                }
            }
        }
        return dist;
    }

    // vector< vector<int> > flloydWarshel(){
        
    // }

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

        if(rank[pu] < rank[pv]){
            parent[pu] = pv;
        }
        else if(rank[pv] < rank[pu]){
            parent[pv] = pu;
        }
        else{
            parent[pv] = pu;
            rank[pu]++;
        }
    }
    // There are two algo for finding Minimun Spanning tree 
    // 1. Prism algo
    // 2. Kruskals algo 

    vector< G > prism_Algo(){
        priority_queue< pair<G,G>, vector< pair<G,G> >, greater< pair<G,G> > > pq;
        vector<G> key(n,INT_MAX);
        vector<bool> mst(n,false);
        pq.push(make_pair(0,0));
        key[0] = 0;
        for(int c=0;c<n-1;c++){
            G node = pq.top().second;
            pq.pop();
            mst[node] = true;
            for(auto it : adj1[node]){
                G curr = it.first;
                L wt = it.second;
                if(mst[curr]==false && wt<key[curr]){
                    pq.push(make_pair(wt,curr));
                    key[curr] = wt;
                }
            }
        }
        return key;
    }

    L kurskals_Algo(){
        vector< pair< L,pair<G,G> > > edges;
        vector<G> parent(n,-1);
        vector<G> rank(n,0);
        L cost = 0;
        for(int i=0;i<n;i++){
            for(auto it : adj1[i]){
                edges.push_back(make_pair(it.second,make_pair(i,it.first)));
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
        return cost;
    }

    vector< pair<G,G> > bridgesInGraph(G node = 0){
        vector<G> dis(n,0);
        vector<G> low(n,0);
        vector<G> vis(n,0);
        vector< pair<G,G> > ans;
        int count = 0;
        for(int i=0;i<n;i++)
        {
            if(!vis[i])
            {
                bridges(dis,low,vis,ans,i,-1,count);
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
            int count = 0;
            if(!vis[i])
            {
                articulation(dis,low,vis,ans,i,-1,count);
            }
        }
        return ans;
    }

};

int main(){
    cout<<"To use this library create an object of class and call the desired function";
    // Graph graph();
    // int nodes, edges;
    // cout<<"Enter number of Edges"<<endl;
    // cin>>edges;
    // cout<<endl;
    // cout<<"Enter number of Nodes"<<endl;
    // cin>>nodes;
    // cout<<endl;
    // vector< vector<int> > adj(edges,vector<int>(2));
    // cout<<"Enter the Edges"<<endl;
    // for(int i=0;i<edges;i++){
    //     int x, y;
    //     cin>>x>>y;
    //     adj[i][0] = x;
    //     adj[i][1] = y;
    // }
    // vector< vector<int> > adjList(nodes);
    // for(int i=0;i<edges;i++){
    //     adjList[adj[i][0]].push_back(adj[i][1]);
    //     // adjList[adj[i][1]].push_back(adj[i][0]);
    // }
    // vector<long long>wt(edges,1);

    // Graph<int,long long> graph(adj,nodes,wt,1);

    // // vector<int> bfs = graph.breadthFirstSearch();
    // // vector<int> dfs = graph.depthFirstSearch();

    // // for(int it : bfs){
    // //     cout<<it<<" ";
    // // }
    // // cout<<endl;
    // // for(int it : dfs){
    // //     cout<<it<<" ";
    // // }

    // // vector<int> topo = graph.toposort();
    // // for(auto it : topo) cout<<it<<" ";


    // // bool isCycle = graph.detectCycleInDirectedGraph();
    // // cout<<isCycle;

    // // bool isBiparitite = graph.checkBipartiteGraph();
    // // cout<<isBiparitite<<endl;

    // // cout<<"Enter the weight"<<endl;
    // // for(int i=0;i<edges;i++)
    // // {
    // //     cin>>wt[i];
    // // }

    // // Graph<int, long long>graph(adj, nodes, wt, 1);
    // // vector<int>no = graph.dijastra();
    // // vector<int>no = graph.bellmanFord();
    // // cout<<"Nodes\n";
    // // for(auto it:no)
    // // cout<<it<<" ";

    // // vector<int> parent(nodes);
    // // vector<int> rank(nodes,0);
    // // for(int i=0;i<nodes;i++){
    // //     parent[i] = i;
    // // }    
    // // for(int i=0;i<adj.size();i++){
    // //     graph.unionFind(adj[i][0],adj[i][1],parent,rank);
    // // }

    // // vector< int > prism = graph.prism_Algo();
    // // for(auto it : prism){
    // //     cout<<it<<" ";
    // // }

    // // int cost  = graph.kurskals_Algo();
    // // cout<<cost;

    // // vector< pair<int,int> > bridge = graph.bridgesInGraph();
    // // for(auto it : bridge){
    // //     cout<<it.first<<" "<<it.second<<endl;
    // // }

    // vector< int > bridge = graph.articulationPoint();
    // for(auto it : bridge){
    //     cout<<it<<endl;
    // }

}

