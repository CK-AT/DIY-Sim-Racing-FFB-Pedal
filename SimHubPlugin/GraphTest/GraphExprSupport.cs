using System;
using System.Collections.Generic;
using System.Globalization;
using NCalcExpression = NCalc.Expression;

namespace DiyFfb.GraphTest
{
    /// <summary>
    /// Shared NCalc support for Expr nodes. Centralises parsing, identifier
    /// extraction, and result coercion so the runtime evaluators and the
    /// validator stay consistent.
    ///
    /// Expr nodes evaluate a user-authored formula whose only free identifiers
    /// are the names of the node's wired inports (resolved from InputMap). Built-in
    /// NCalc functions (Pow, Sin, Abs, if(...), etc.) are allowed; they are NOT
    /// treated as identifiers, so the inport-scope rule does not block them.
    /// </summary>
    internal static class GraphExprSupport
    {
        // Formulas are authored in invariant culture ("1.5", not "1,5") so saved
        // graphs evaluate identically regardless of the host machine's locale.
        public static readonly CultureInfo Culture = CultureInfo.InvariantCulture;

        // Built-in named constants. These are allowed in formulas regardless of the
        // inport-scope rule, and seeded into the expression's parameters once.
        public static readonly Dictionary<string, double> Constants =
            new Dictionary<string, double>(StringComparer.Ordinal)
            {
                ["Pi"] = Math.PI,
            };

        public static bool IsConstant(string name) => name != null && Constants.ContainsKey(name);

        /// <summary>Seeds any built-in constants referenced by the formula. Constant
        /// values never change, so this is called once (not per evaluation).</summary>
        public static void SeedConstants(NCalcExpression expr, IEnumerable<string> usedNames)
        {
            if (expr == null || usedNames == null) return;
            foreach (var name in usedNames)
            {
                if (Constants.TryGetValue(name, out var value))
                {
                    expr.Parameters[name] = value;
                }
            }
        }

        /// <summary>
        /// Parses a formula. On success returns the (parsed) Expression and sets
        /// error to null. On failure returns null and sets a human-readable error.
        /// </summary>
        public static NCalcExpression TryParse(string text, out string error)
        {
            error = null;
            if (string.IsNullOrWhiteSpace(text))
            {
                error = "expression is empty";
                return null;
            }

            NCalcExpression expr;
            try
            {
                expr = new NCalcExpression(text, NCalc.EvaluateOptions.None, Culture);
                if (expr.HasErrors())
                {
                    error = expr.Error;
                    return null;
                }
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return null;
            }
            return expr;
        }

        /// <summary>
        /// Returns the set of free identifiers (parameter references) in a parsed
        /// expression. Function names are excluded — only operands count.
        /// </summary>
        public static HashSet<string> CollectIdentifiers(NCalcExpression expr)
        {
            var collector = new IdentifierCollector();
            // Accessing ParsedExpression after a successful HasErrors() yields the AST.
            expr?.ParsedExpression?.Accept(collector);
            return collector.Names;
        }

        /// <summary>Coerces an NCalc evaluation result (object) to double.</summary>
        public static double ToDouble(object value)
        {
            if (value == null) return 0.0;
            if (value is double d) return d;
            if (value is bool b) return b ? 1.0 : 0.0;
            try
            {
                return Convert.ToDouble(value, Culture);
            }
            catch
            {
                return 0.0;
            }
        }

        /// <summary>
        /// Walks the NCalc AST collecting standalone identifier names. A Function's
        /// own Identifier (its name) is deliberately not collected — only its
        /// argument expressions are visited — so "Pow(x, 2)" reports {x}, not {Pow}.
        /// </summary>
        private sealed class IdentifierCollector : NCalc.Domain.LogicalExpressionVisitor
        {
            public readonly HashSet<string> Names = new HashSet<string>(StringComparer.Ordinal);

            public override void Visit(NCalc.Domain.Identifier identifier)
            {
                if (!string.IsNullOrEmpty(identifier.Name))
                {
                    Names.Add(identifier.Name);
                }
            }

            public override void Visit(NCalc.Domain.LogicalExpression expression)
            {
                // Abstract base overload; concrete nodes dispatch to the specific
                // overloads via Accept, so this is not normally reached.
            }

            public override void Visit(NCalc.Domain.UnaryExpression expression)
            {
                expression.Expression.Accept(this);
            }

            public override void Visit(NCalc.Domain.BinaryExpression expression)
            {
                expression.LeftExpression.Accept(this);
                expression.RightExpression.Accept(this);
            }

            public override void Visit(NCalc.Domain.TernaryExpression expression)
            {
                expression.LeftExpression.Accept(this);
                expression.MiddleExpression.Accept(this);
                expression.RightExpression.Accept(this);
            }

            public override void Visit(NCalc.Domain.Function function)
            {
                // Skip function.Identifier (the function name); visit only its args.
                if (function.Expressions != null)
                {
                    foreach (var arg in function.Expressions)
                    {
                        arg.Accept(this);
                    }
                }
            }

            public override void Visit(NCalc.Domain.ValueExpression expression)
            {
                // Literal — no identifiers.
            }
        }
    }
}
