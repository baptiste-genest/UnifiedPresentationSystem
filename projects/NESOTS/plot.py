import marimo

__generated_with = "0.1.63"
app = marimo.App()


@app.cell
def __():
    import numpy as np
    import matplotlib.pyplot as plt
    return np, plt


@app.cell
def __(np, plt):
    # plot 2D gaussian density between -3 and 3
    x = np.linspace(-3, 3, 200)
    y = np.linspace(-3, 3, 200)
    X, Y = np.meshgrid(x, y)
    Z = np.exp(-0.5*(X**2 + Y**2))

    # sample points from 2D gaussian density
    N = 1000
    PX = np.random.randn(N)
    PY = np.random.randn(N)

    #split figure in two
    fig, ax = plt.subplots(1, 2, figsize=(10, 5))

    # plot density on the left with imshow
    ax[0].imshow(Z, extent=(-3, 3, -3, 3), origin='lower')
    ax[0].set_xlim(-3, 3)
    ax[0].set_ylim(-3, 3)

    # plot sample on the right
    ax[1].scatter(PX, PY, alpha=0.5)
    ax[1].set_xlim(-3, 3)
    ax[1].set_ylim(-3, 3)

    # remove ticks and border 
    for a in ax:
        a.set_xticks([])
        a.set_yticks([])
        a.spines['top'].set_visible(False)
        a.spines['right'].set_visible(False)
        a.spines['bottom'].set_visible(False)
        a.spines['left'].set_visible(False)




    #plot 2D density without contour


    plt.show()
    return N, PX, PY, X, Y, Z, a, ax, fig, x, y


@app.cell
def __(X, np, plt):
    # plot 2D uniform density in cyan
    U = np.ones_like(X)
    plt.imshow(U, extent=(-3, 3, -3, 3), origin='lower', cmap='cool')

    return U,


@app.cell
def __():
    return


if __name__ == "__main__":
    app.run()
