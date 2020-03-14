using System;

namespace VectorNav.Math
{
/// <summary>
/// Represents a vector of dimension 7 with DOP data values
/// type of <c>float</c>.
/// </summary>
// ReSharper disable once InconsistentNaming
public struct vecDop
{
	#region Public Members

	/// <summary>
	/// The gDOP value.
	/// </summary>
	public float G;

  /// <summary>
  /// The pDOP value.
  /// </summary>
  public float P;

  /// <summary>
  /// The tDOP value.
  /// </summary>
  public float T;

  /// <summary>
  /// The vDOP value.
  /// </summary>
  public float V;

  /// <summary>
  /// The hDOP value.
  /// </summary>
  public float H;

  /// <summary>
  /// The nDOP value.
  /// </summary>
  public float N;

  /// <summary>
  /// The eDOP value.
  /// </summary>
  public float E;

  #endregion

  #region Public Properties

    /// <summary>
    /// The dimensions of the <see cref="vecDop"/>.
    /// </summary>
    public int Dimension { get { return 7; } }

	#endregion

	#region Object Overrides

	/// <inheritdoc/>
	public override string ToString()
	{
		return string.Format(
      "({0}; {1}; {2}; {3}; {4}; {5}; {6})",
			G,
			P,
			T,
      V,
      H,
      N,
      E);
	}

	#endregion

	#region Constructors

	/// <summary>
	/// Creates a new <see cref="vecDop/> with default values
		/// </summary>
	/// <param name="value">
	/// The value to initialize the DOP components with.
	/// </param>
	public vecDop(float value)
	{
		G = value;
    P = value;
    T = value;
    V = value;
    H = value;
    N = value;
    E = value;
  }

  /// <summary>
  /// Creates a new <see cref="vecDop"/> with the provided values for its 
  /// components.
  /// </summary>
  /// <param name="g">
  /// The value for the G (0) component.
  /// </param>
  /// <param name="p">
  /// The value for the P (1) component.
  /// </param>
  /// <param name="t">
  /// The value for the T (2) component.
  /// </param>
  /// <param name="v">
  /// The value for the V (3) component.
  /// </param>
  /// <param name="h">
  /// The value for the H (4) component.
  /// </param>
  /// <param name="n">
  /// The value for the N (5) component.
  /// </param>
  /// <param name="e">
  /// The value for the E (6) component.
  /// </param>
  public vecDop(float g, float p, float t, float v, float h, float n, float e)
	{
		G = g;
		P = p;
		T = t;
		V = v;
    H = h;
    N = n;
    E = e;
  }

  #endregion

  #region Operator Overloads

  /// <summary>
  /// Allows indexing into the <see cref="vecDop"/>.
  /// </summary>
  /// <param name="index">
  /// The specified index. Must be in the range [0, 6].
  /// </param>
  /// <returns>
  /// The value at the specified index.
  /// </returns>
  /// <exception cref="IndexOutOfRangeException">
  /// Thrown if the specified index is out of range.
  /// </exception>
  public float this[int index]
	{
		get
		{
			switch (index)
			{
				case 0:
					return G;

				case 1:
					return P;

				case 2:
					return T;

				case 3:
					return V;

        case 4:
          return H;

        case 5:
          return N;

        case 6:
          return E;

        default:
					return vecf.ThrowInvalidIndexDetected(index, 7);
			}
		}
		set
		{
			switch (index)
			{
				case 0:
          G = value;
					break;

				case 1:
					P = value;
					break;

				case 2:
					T = value;
					break;

				case 3:
					V = value;
					break;

        case 4:
          H = value;
          break;

        case 5:
          N = value;
          break;

        case 6:
          E = value;
          break;

        default:
					vecf.ThrowInvalidIndexDetected(index, 4);
					break;
			}
		}
	}

	#endregion

	#region Public Methods

	/// <summary>
	/// Returns an array of the <see cref="vec4f"/>'s component values.
	/// </summary>
	/// <returns>
	/// The array containing the <see cref="vec4f"/> components.
	/// </returns>
	public float[] ToArray()
	{
		return new[] { G, P, T, V, H, N, E };
	}

	#endregion

}
}
