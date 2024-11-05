using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.IO;

class Agent : BaseAgent
{

    [STAThread]
    static void Main(){
        Program.Start(new Agent());
    }
    public Agent() { }
    public override Drag SökNästaDrag(SpelBräde bräde){
        Spelare jag = bräde.spelare[0];
        Spelare motståndare = bräde.spelare[1];
        Point myPosition = jag.position;
        Point opponentPosition = motståndare.position;

        List<Point> myPath = BFS(bräde, myPosition, new Point(myPosition.X, SpelBräde.N - 1)); // AI goal at the top
        List<Point> opponentPath = BFS(bräde, opponentPosition, new Point(opponentPosition.X, 0)); // Opponent goal at the bottom

        Drag drag = new Drag();

        if (myPath.Count > 1 && opponentPath.Count > 1)
        {
            if (myPath.Count <= opponentPath.Count)
            {
                // Move if AI path is shorter or equal
                drag = MoveAlongPath(myPath);
            }
            else if (opponentPath.Count < myPath.Count && jag.antalVäggar > 0)
            {
                // Block position in front of the opponent
                Point blockPosition = opponentPath[1];

                // Determine opponent's movement direction (forward movement)
                Point movementDirection = GetMovementDirection(opponentPosition, blockPosition);

                // Try placing a wall based on the opponent's forward movement
                if (movementDirection.Y < 0)
                { // Opponent is moving upward (toward their goal)
                    if (CanPlaceHorizontalWall(bräde, blockPosition))
                    {
                        PlaceHorizontalWall(bräde, blockPosition, opponentPosition, ref drag);
                    }
                }
                else if (movementDirection.Y > 0)
                { // Opponent is moving downward (for some reason)
                    if (CanPlaceHorizontalWall(bräde, blockPosition))
                    {
                        PlaceHorizontalWall(bräde, blockPosition, opponentPosition, ref drag);
                    }
                }
                else if (movementDirection.X < 0)
                { // Opponent is moving left
                    if (CanPlaceVerticalWall(bräde, blockPosition))
                    {
                        PlaceVerticalWall(bräde, blockPosition, opponentPosition, ref drag);
                    }
                }
                else if (movementDirection.X > 0)
                { // Opponent is moving right
                    if (CanPlaceVerticalWall(bräde, blockPosition))
                    {
                        PlaceVerticalWall(bräde, blockPosition, opponentPosition, ref drag);
                    }
                }

                // Try pushing the wall forward or backward
                if (drag.typ == Typ.Flytta && CanPushHorizontalWallForward(bräde, blockPosition, opponentPosition))
                {
                    PushHorizontalWallForward(bräde, blockPosition, opponentPosition, ref drag);
                }
                else if (drag.typ == Typ.Flytta && CanPushHorizontalWallBackward(bräde, blockPosition, opponentPosition))
                {
                    PushHorizontalWallBackward(bräde, blockPosition, opponentPosition, ref drag);
                }

                // Try pushing the vertical wall in both directions (upward and downward)
                if (drag.typ == Typ.Flytta && CanPushVerticalWallForward(bräde, blockPosition, opponentPosition))
                {
                    PushVerticalWallForward(bräde, blockPosition, opponentPosition, ref drag);
                }
                else if (drag.typ == Typ.Flytta && CanPushVerticalWallBackward(bräde, blockPosition, opponentPosition))
                {
                    PushVerticalWallBackward(bräde, blockPosition, opponentPosition, ref drag);
                }

                // If no wall placement works, move along AI path
                if (drag.typ == Typ.Flytta)
                {
                    drag = MoveAlongPath(myPath);
                }
            }
            else
            {
                drag = MoveAlongPath(myPath);
            }
        }
        else
        {
            drag.typ = Typ.Flytta;
            drag.point = new Point(myPosition.X, myPosition.Y + 1); // Fallback move
        }

        return drag; // Return the decision
    }

    // Helper method to determine the movement direction of the opponent
    private Point GetMovementDirection(Point currentPos, Point nextPos)
    {
        // Calculate the difference between current and next positions to get movement direction
        int deltaX = nextPos.X - currentPos.X;
        int deltaY = nextPos.Y - currentPos.Y;
        return new Point(deltaX, deltaY);
    }


    // Helper method to move the AI along a calculated path
    private Drag MoveAlongPath(List<Point> path)
    {
        Drag drag = new Drag();
        if (path.Count > 1)
        {
            // Move to the next point in the path
            drag.typ = Typ.Flytta;
            drag.point = path[1];
        }
        else
        {
            // If only one point exists, stay in place
            drag.typ = Typ.Flytta;
            drag.point = path[0];
        }
        return drag;
    }

    // Helper method to place a horizontal wall
    private void PlaceHorizontalWall(SpelBräde bräde, Point blockPosition, Point opponentPosition, ref Drag drag)
    {
        // Place the two parts of a horizontal wall (two squares long)
        bräde.horisontellaVäggar[blockPosition.X, blockPosition.Y] = true;
        bräde.horisontellaVäggar[blockPosition.X + 1, blockPosition.Y] = true;

        // Check if the opponent still has a valid path
        if (OpponentHasPath(bräde, opponentPosition))
        {
            // If valid, place the wall and update the drag
            drag.typ = Typ.Horisontell;
            drag.point = blockPosition;
        }
        else
        {
            // If not valid, undo the wall placement
            bräde.horisontellaVäggar[blockPosition.X, blockPosition.Y] = false;
            bräde.horisontellaVäggar[blockPosition.X + 1, blockPosition.Y] = false;
        }
    }

    // Helper method to push a horizontal wall forward (right)
    private void PushHorizontalWallForward(SpelBräde bräde, Point blockPosition, Point opponentPosition, ref Drag drag)
    {
        // Place the pushed horizontal wall (shifted one square to the right)
        bräde.horisontellaVäggar[blockPosition.X + 1, blockPosition.Y] = true;
        bräde.horisontellaVäggar[blockPosition.X + 2, blockPosition.Y] = true;

        // Check if the opponent still has a valid path
        if (OpponentHasPath(bräde, opponentPosition))
        {
            // If valid, place the pushed wall and update the drag
            drag.typ = Typ.Horisontell;
            drag.point = new Point(blockPosition.X + 1, blockPosition.Y); // Adjusted point to match pushed wall
        }
        else
        {
            // If not valid, undo the pushed wall placement
            bräde.horisontellaVäggar[blockPosition.X + 1, blockPosition.Y] = false;
            bräde.horisontellaVäggar[blockPosition.X + 2, blockPosition.Y] = false;
        }
    }

    // Helper method to push a horizontal wall backward (left)
    private void PushHorizontalWallBackward(SpelBräde bräde, Point blockPosition, Point opponentPosition, ref Drag drag)
    {
        // Place the pushed horizontal wall (shifted one square to the left)
        bräde.horisontellaVäggar[blockPosition.X - 1, blockPosition.Y] = true;
        bräde.horisontellaVäggar[blockPosition.X, blockPosition.Y] = true;

        // Check if the opponent still has a valid path
        if (OpponentHasPath(bräde, opponentPosition))
        {
            // If valid, place the pushed wall and update the drag
            drag.typ = Typ.Horisontell;
            drag.point = new Point(blockPosition.X - 1, blockPosition.Y); // Adjusted point to match pushed wall
        }
        else
        {
            // If not valid, undo the pushed wall placement
            bräde.horisontellaVäggar[blockPosition.X - 1, blockPosition.Y] = false;
            bräde.horisontellaVäggar[blockPosition.X, blockPosition.Y] = false;
        }
    }

    // Helper method to place a vertical wall
    private void PlaceVerticalWall(SpelBräde bräde, Point blockPosition, Point opponentPosition, ref Drag drag)
    {
        // Place the two parts of a vertical wall (two squares long)
        bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y] = true;
        bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y + 1] = true;

        // Check if the opponent still has a valid path
        if (OpponentHasPath(bräde, opponentPosition))
        {
            // If valid, place the wall and update the drag
            drag.typ = Typ.Vertikal;
            drag.point = blockPosition;
        }
        else
        {
            // If not valid, undo the wall placement
            bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y] = false;
            bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y + 1] = false;
        }
    }

    // Helper method to push a vertical wall forward (upward)
    private void PushVerticalWallForward(SpelBräde bräde, Point blockPosition, Point opponentPosition, ref Drag drag)
    {
        // Place the pushed vertical wall (shifted one square upward)
        bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y + 1] = true;
        bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y + 2] = true;

        // Check if the opponent still has a valid path
        if (OpponentHasPath(bräde, opponentPosition))
        {
            // If valid, place the pushed wall and update the drag
            drag.typ = Typ.Vertikal;
            drag.point = new Point(blockPosition.X, blockPosition.Y + 1); // Adjusted point to match pushed wall
        }
        else
        {
            // If not valid, undo the pushed wall placement
            bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y + 1] = false;
            bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y + 2] = false;
        }
    }

    // Helper method to push a vertical wall backward (downward)
    private void PushVerticalWallBackward(SpelBräde bräde, Point blockPosition, Point opponentPosition, ref Drag drag)
    {
        // Place the pushed vertical wall (shifted one square downward)
        bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y - 1] = true;
        bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y] = true;

        // Check if the opponent still has a valid path
        if (OpponentHasPath(bräde, opponentPosition))
        {
            // If valid, place the pushed wall and update the drag
            drag.typ = Typ.Vertikal;
            drag.point = new Point(blockPosition.X, blockPosition.Y - 1); // Adjusted point to match pushed wall
        }
        else
        {
            // If not valid, undo the pushed wall placement
            bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y - 1] = false;
            bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y] = false;
        }
    }

    // Check if the opponent still has a path after placing a wall
    private bool OpponentHasPath(SpelBräde bräde, Point opponentPosition)
    {
        // Run BFS to check if the opponent has a valid path to their goal
        List<Point> opponentPath = BFS(bräde, opponentPosition, new Point(opponentPosition.X, 0));
        return opponentPath.Count > 1; // Return true if a valid path exists
    }

    // Check if a horizontal wall can be placed at the specified position
    private bool CanPlaceHorizontalWall(SpelBräde bräde, Point position)
    {
        // Check if a horizontal wall (two squares long) can be placed
        return position.Y < SpelBräde.N - 1 && position.X < SpelBräde.N - 2 &&
               !bräde.horisontellaVäggar[position.X, position.Y] &&
               !bräde.horisontellaVäggar[position.X + 1, position.Y];
    }

    // Check if a vertical wall can be placed at the specified position
    private bool CanPlaceVerticalWall(SpelBräde bräde, Point position)
    {
        // Check if a vertical wall (two squares long) can be placed
        return position.X < SpelBräde.N - 1 && position.Y < SpelBräde.N - 2 &&
               !bräde.vertikalaVäggar[position.X, position.Y] &&
               !bräde.vertikalaVäggar[position.X, position.Y + 1];
    }

    // Check if a horizontal wall can be pushed forward
    private bool CanPushHorizontalWallForward(SpelBräde bräde, Point blockPosition, Point opponentPosition)
    {
        // Check if the horizontal wall can be pushed one square to the right
        return blockPosition.X + 2 < SpelBräde.N - 1 &&
               !bräde.horisontellaVäggar[blockPosition.X + 1, blockPosition.Y] &&
               !bräde.horisontellaVäggar[blockPosition.X + 2, blockPosition.Y];
    }

    // Check if a horizontal wall can be pushed backward
    private bool CanPushHorizontalWallBackward(SpelBräde bräde, Point blockPosition, Point opponentPosition)
    {
        // Check if the horizontal wall can be pushed one square to the left
        return blockPosition.X - 1 >= 0 &&
               !bräde.horisontellaVäggar[blockPosition.X - 1, blockPosition.Y] &&
               !bräde.horisontellaVäggar[blockPosition.X, blockPosition.Y];
    }

    // Check if a vertical wall can be pushed forward (upward)
    private bool CanPushVerticalWallForward(SpelBräde bräde, Point blockPosition, Point opponentPosition)
    {
        // Check if the vertical wall can be pushed one square upward
        return blockPosition.Y + 2 < SpelBräde.N - 1 &&
               !bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y + 1] &&
               !bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y + 2];
    }

    // Check if a vertical wall can be pushed backward (downward)
    private bool CanPushVerticalWallBackward(SpelBräde bräde, Point blockPosition, Point opponentPosition)
    {
        // Check if the vertical wall can be pushed one square downward
        return blockPosition.Y - 1 >= 0 &&
               !bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y - 1] &&
               !bräde.vertikalaVäggar[blockPosition.X, blockPosition.Y];
    }

    // Breadth-First Search (BFS) to calculate the shortest path
    private List<Point> BFS(SpelBräde bräde, Point start, Point goal)
    {
        Queue<Point> queue = new Queue<Point>();
        Dictionary<Point, Point> cameFrom = new Dictionary<Point, Point>();
        queue.Enqueue(start);
        cameFrom[start] = start;

        while (queue.Count > 0)
        {
            Point current = queue.Dequeue();
            if (current == goal)
            {
                return ReconstructPath(cameFrom, start, goal);
            }
            foreach (Point neighbor in GetNeighbors(bräde, current))
            {
                if (!cameFrom.ContainsKey(neighbor))
                {
                    queue.Enqueue(neighbor);
                    cameFrom[neighbor] = current;
                }
            }
        }
        return new List<Point> { start };
    }

    // Reconstruct the shortest path from the cameFrom dictionary
    private List<Point> ReconstructPath(Dictionary<Point, Point> cameFrom, Point start, Point goal)
    {
        List<Point> path = new List<Point>();
        Point current = goal;
        while (current != start)
        {
            path.Add(current);
            current = cameFrom[current];
        }
        path.Add(start);
        path.Reverse();
        return path;
    }

    // Get the valid neighboring positions considering walls and boundaries
    private List<Point> GetNeighbors(SpelBräde bräde, Point current)
    {
        List<Point> neighbors = new List<Point>();

        // Check left, right, above, and below, considering walls
        if (current.X > 0 && !bräde.vertikalaVäggar[current.X - 1, current.Y])
            neighbors.Add(new Point(current.X - 1, current.Y)); // Left
        if (current.X < SpelBräde.N - 1 && !bräde.vertikalaVäggar[current.X, current.Y])
            neighbors.Add(new Point(current.X + 1, current.Y)); // Right
        if (current.Y > 0 && !bräde.horisontellaVäggar[current.X, current.Y - 1])
            neighbors.Add(new Point(current.X, current.Y - 1)); // Down
        if (current.Y < SpelBräde.N - 1 && !bräde.horisontellaVäggar[current.X, current.Y])
            neighbors.Add(new Point(current.X, current.Y + 1)); // Up

        return neighbors;
    }
}
