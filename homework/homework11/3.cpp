#include <bits/stdc++.h>
#include <cmath>
#include <functional>
using namespace std;

typedef struct node
{
    int num;
    node *l;
    node *r;
    node(int n)
    {
        num = n;
        l = NULL;
        r = NULL;
    }
} node;

node *solve(node *root, int l, int r)
{
    int sum = r - l + 1;
    if (sum == 3)
    {
        root = new node(l + 1);
        root->l = new node(l);
        root->r = new node(r);
        return root;
    }
    else if (sum == 2)
    {
        root = new node(l);
        root->r = new node(r);
        return root;
    }
    else if (sum == 1)
    {
        root = new node(l);
        return root;
    }
    if (log2(sum + 1) == (int)log2(sum + 1))
    {
        root = new node((l + r) / 2);
        root->l = solve(root->l, l, (l + r) / 2 - 1);
        root->r = solve(root->r, (l + r) / 2 + 1, r);
    }
    else
    {
        int depth = log2(sum);
        int mid = r - pow(2, depth) + 1;
        root = new node(mid);
        root->l = solve(root->l, l, mid - 1);
        root->r = solve(root->r, mid + 1, r);
    }
    return root;
}

void frontprint(node *root)
{
    if (root == NULL)
        return;
    cout << root->num << " ";
    frontprint(root->l);
    frontprint(root->r);
}

int main()
{
    int n;
    cin >> n;
    node *root = NULL;
    root = solve(root, 1, n);
    int depth = log2(n + 1);
    printf("%d\n", depth);
    frontprint(root);
    return 0;
}